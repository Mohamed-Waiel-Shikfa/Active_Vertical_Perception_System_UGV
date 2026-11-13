// Lichtblick panel "Subsystem graph": the ROS graph of one robot subsystem at a time.
// Data: tools/ros_graph.py on /greenhouse/viz/graph. Rates: messages this panel receives.
import ELK from "elkjs/lib/elk.bundled.js";

import {
  GRAPH_TOPIC, NODE_HEIGHT, RateMeter, TOPIC_HEIGHT, edgePath, elkGraph, shortName, shortType, view, viewKey,
} from "./graph.js";

const SVG = "http://www.w3.org/2000/svg";
const MIN_FIT = 0.45; // fit never zooms out further: names stay readable, the rest pans
const FONT = "12px system-ui, -apple-system, 'Segoe UI', sans-serif";
const MONO = "12px ui-monospace, 'SF Mono', 'JetBrains Mono', Menlo, monospace";
const THEMES = {
  dark: { text: "#e3e5ea", muted: "#8b909c", node: "#262931", nodeEdge: "#454a57", topic: "#15171c",
          live: "#4fb3a0", edge: "#5d6370", hot: "#e8b04a", tab: "#2b2e36" },
  light: { text: "#1d2026", muted: "#6b7180", node: "#ffffff", nodeEdge: "#c3c7cf", topic: "#f4f5f7",
           live: "#16846f", edge: "#a3a8b2", hot: "#b7791f", tab: "#e7e9ed" },
};
const CSS = `
.sg{position:relative;width:100%;height:100%;display:flex;flex-direction:column;font:${FONT};color:var(--text);user-select:none}
.sg-bar{display:flex;gap:4px;padding:6px 8px;align-items:center;overflow-x:auto;scrollbar-width:thin;flex:none;border-bottom:1px solid var(--nodeEdge)}
.sg-tab{flex:none;white-space:nowrap;font:inherit;color:var(--muted);background:none;border:1px solid transparent;border-radius:6px;padding:3px 9px;cursor:pointer}
.sg-tab:hover{color:var(--text)}
.sg-tab[aria-selected=true]{color:var(--text);background:var(--tab);border-color:var(--nodeEdge)}
.sg-tab span{margin-left:6px;opacity:.6;font-variant-numeric:tabular-nums}
.sg-tab:focus-visible,.sg-fit:focus-visible{outline:2px solid var(--hot);outline-offset:1px}
.sg-fit.sg-next{margin-left:6px}
.sg-fit{flex:none;margin-left:auto;font:inherit;color:var(--muted);background:none;border:1px solid var(--nodeEdge);border-radius:6px;padding:3px 9px;cursor:pointer}
.sg-stage{position:relative;flex:1;overflow:hidden;cursor:grab}
.sg-stage:active{cursor:grabbing}
.sg-msg{position:absolute;inset:0;display:grid;place-items:center;color:var(--muted);text-align:center;padding:16px}
.sg svg{width:100%;height:100%;display:block}
.sg .edge{fill:none;stroke:var(--edge);stroke-width:1.25}
.sg .box rect{stroke-width:1}
.sg .node rect{fill:var(--node);stroke:var(--nodeEdge)}
.sg .topic rect{fill:var(--topic);stroke:var(--nodeEdge)}
.sg .topic.live rect{stroke:var(--live)}
.sg .topic.orphan rect{stroke-dasharray:3 3}
.sg .name{fill:var(--text)}
.sg .topic .name{font:${MONO}}
.sg .meta{fill:var(--muted);font-size:11px;font-variant-numeric:tabular-nums}
.sg .topic.live .rate{fill:var(--live)}
.sg svg.focus .edge,.sg svg.focus .box{opacity:.18}
.sg svg.focus .hot{opacity:1}
.sg svg.focus .edge.hot{stroke:var(--hot);stroke-width:1.75}
.sg svg.focus .box.hot rect{stroke:var(--hot)}
@media (prefers-reduced-motion:no-preference){.sg .edge,.sg .box{transition:opacity .12s ease-out}}
`;

function initPanel(context) {
  const elk = new ELK();
  const meter = new RateMeter();
  const ctx2d = document.createElement("canvas").getContext("2d");
  const measure = (text, mono) => ((ctx2d.font = mono ? MONO : FONT), Math.ceil(ctx2d.measureText(text).width));
  let subsystem = context.initialState?.subsystem ?? "drive";
  let showUnused = context.initialState?.showUnused ?? false;
  let graph;
  let graphText = "";
  let layoutKey = "";
  let layoutToken = 0;
  let layoutDirection = "";
  let subscribed = "";
  let rateNodes = new Map(); // topic -> [group, rate text]
  let pan = { x: 0, y: 0, k: 1 };
  let size = { width: 0, height: 0 };

  const root = document.createElement("div");
  root.className = "sg";
  root.innerHTML = `<style>${CSS}</style><div class="sg-bar" role="tablist"></div>
    <div class="sg-stage"><svg><defs><marker id="sg-arrow" viewBox="0 0 8 8" refX="7" refY="4" markerWidth="7"
    markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L8,4 L0,8 z" fill="var(--edge)"/></marker></defs>
    <g class="world"></g></svg><div class="sg-msg">Waiting for ${GRAPH_TOPIC}. Start the sim with run.sh --viz.</div></div>`;
  context.panelElement.appendChild(root);
  const bar = root.querySelector(".sg-bar");
  const stage = root.querySelector(".sg-stage");
  const svg = root.querySelector("svg");
  const world = root.querySelector(".world");
  const message = root.querySelector(".sg-msg");

  const applyPan = () => world.setAttribute("transform", `translate(${pan.x},${pan.y}) scale(${pan.k})`);
  function fit() {
    const r = stage.getBoundingClientRect();
    if (!size.width || !r.width) return;
    const k = Math.max(MIN_FIT, Math.min(1.25, r.width / size.width, r.height / size.height));
    pan = { k, x: Math.max(0, (r.width - size.width * k) / 2), y: Math.max(0, (r.height - size.height * k) / 2) };
    applyPan();
  }

  stage.addEventListener("wheel", (e) => {
    e.preventDefault();
    const r = stage.getBoundingClientRect();
    const [mx, my] = [e.clientX - r.left, e.clientY - r.top];
    const k = Math.min(4, Math.max(0.2, pan.k * Math.exp(-e.deltaY * 0.0015)));
    pan = { k, x: mx - ((mx - pan.x) * k) / pan.k, y: my - ((my - pan.y) * k) / pan.k };
    applyPan();
  }, { passive: false });
  stage.addEventListener("pointerdown", (e) => {
    const start = { x: e.clientX - pan.x, y: e.clientY - pan.y };
    stage.setPointerCapture(e.pointerId);
    const move = (m) => ((pan = { ...pan, x: m.clientX - start.x, y: m.clientY - start.y }), applyPan());
    const up = () => (stage.removeEventListener("pointermove", move), stage.removeEventListener("pointerup", up));
    stage.addEventListener("pointermove", move);
    stage.addEventListener("pointerup", up);
  });
  stage.addEventListener("dblclick", fit);
  // Wide panels lay out left to right, tall ones top to bottom.
  const direction = () => (stage.clientWidth >= stage.clientHeight ? "RIGHT" : "DOWN");
  new ResizeObserver(() => (graph && direction() !== layoutDirection ? relayout() : fit())).observe(stage);

  function el(tag, attrs, parent) {
    const node = document.createElementNS(SVG, tag);
    for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
    parent.appendChild(node);
    return node;
  }

  function renderTabs() {
    bar.replaceChildren();
    for (const s of graph.subsystems) {
      const tab = document.createElement("button");
      tab.className = "sg-tab";
      tab.setAttribute("role", "tab");
      tab.setAttribute("aria-selected", String(s.id === subsystem));
      const count = graph.topics.filter((t) => t.subsystems.includes(s.id)).length;
      tab.innerHTML = `${s.label}<span>${count}</span>`;
      tab.onclick = () => {
        subsystem = s.id;
        context.saveState({ subsystem, showUnused });
        renderTabs();
        relayout();
      };
      bar.appendChild(tab);
    }
    const unused = graph.topics.filter((t) => t.subsystems.includes(subsystem) && !t.publishers.length).length;
    if (unused) {
      const toggle = document.createElement("button");
      toggle.className = "sg-fit";
      toggle.setAttribute("aria-pressed", String(showUnused));
      toggle.textContent = `${showUnused ? "Hide" : "Show"} unused inputs (${unused})`;
      toggle.title = "Topics with subscribers but no publisher";
      toggle.onclick = () => {
        showUnused = !showUnused;
        context.saveState({ subsystem, showUnused });
        renderTabs();
        relayout();
      };
      bar.appendChild(toggle);
    }
    const fitButton = document.createElement("button");
    fitButton.className = unused ? "sg-fit sg-next" : "sg-fit";
    fitButton.textContent = "Fit";
    fitButton.title = "Fit to view (double-click the graph)";
    fitButton.onclick = fit;
    bar.appendChild(fitButton);
  }

  function updateSubscriptions(v) {
    const topics = v.topics.filter((t) => t.type !== "action" && t.publishers.length).map((t) => t.name);
    const key = topics.join("\n");
    if (key === subscribed) return;
    subscribed = key;
    context.subscribe([GRAPH_TOPIC, ...topics].map((topic) => ({ topic })));
  }

  async function relayout() {
    const v = view(graph, subsystem, showUnused);
    updateSubscriptions(v);
    layoutDirection = direction();
    const key = subsystem + layoutDirection + viewKey(v);
    if (key === layoutKey) return;
    layoutKey = key;
    const token = ++layoutToken;
    const laid = await elk.layout(elkGraph(v, graph.namespace, measure, layoutDirection));
    if (token !== layoutToken) return;
    draw(v, laid);
  }

  function draw(v, laid) {
    world.replaceChildren();
    rateNodes = new Map();
    size = { width: laid.width, height: laid.height };
    message.style.display = v.topics.length ? "none" : "grid";
    message.textContent = "No topics in this subsystem right now.";
    const boxes = new Map(laid.children.map((c) => [c.id, c]));
    const links = new Map(); // element id -> edges touching it
    const edgeLayer = el("g", {}, world);
    for (const e of laid.edges) {
      const path = el("path", { class: "edge", d: e.sections.map((s) => edgePath(s)).join(" "),
                                "marker-end": "url(#sg-arrow)" }, edgeLayer);
      for (const end of [e.sources[0], e.targets[0]]) links.set(end, [...(links.get(end) ?? []), { path, e }]);
    }
    const groups = new Map();
    for (const n of v.nodes) {
      const b = boxes.get(`n:${n}`);
      const g = el("g", { class: "box node", transform: `translate(${b.x},${b.y})` }, world);
      el("rect", { width: b.width, height: NODE_HEIGHT, rx: 6 }, g);
      el("text", { class: "name", x: 14, y: NODE_HEIGHT / 2 + 4 }, g).textContent = shortName(n, graph.namespace);
      el("title", {}, g).textContent = n;
      groups.set(`n:${n}`, g);
    }
    for (const t of v.topics) {
      const b = boxes.get(`t:${t.name}`);
      const orphan = !t.publishers.length || !t.subscribers.length;
      const g = el("g", { class: `box topic${orphan ? " orphan" : ""}`, transform: `translate(${b.x},${b.y})` }, world);
      el("rect", { width: b.width, height: TOPIC_HEIGHT, rx: 10 }, g);
      el("text", { class: "name", x: 14, y: 15 }, g).textContent = shortName(t.name, graph.namespace);
      const meta = el("text", { class: "meta", x: 14, y: 29 }, g);
      meta.textContent = (t.type === "action" ? "action" : shortType(t.type)) + (t.publishers.length ? "" : "  no publisher");
      el("title", {}, g).textContent = `${t.name}\n${t.type}${t.latched ? " (latched)" : ""}\n`
        + `publishers: ${t.publishers.join(", ") || "none"}\nsubscribers: ${t.subscribers.join(", ") || "none"}\n`
        + "Rate: as received by Lichtblick over the bridge (large images arrive slower than ROS publishes them).";
      if (t.type !== "action" && t.publishers.length) rateNodes.set(t.name, [g, el("tspan", { class: "rate", dx: 8 }, meta), t.latched]);
      groups.set(`t:${t.name}`, g);
    }
    for (const [id, g] of groups) {
      g.addEventListener("pointerenter", () => {
        svg.classList.add("focus");
        g.classList.add("hot");
        for (const { path, e } of links.get(id) ?? []) {
          path.classList.add("hot");
          for (const end of [e.sources[0], e.targets[0]]) groups.get(end)?.classList.add("hot");
        }
      });
      g.addEventListener("pointerleave", () => {
        svg.classList.remove("focus");
        for (const hot of world.querySelectorAll(".hot")) hot.classList.remove("hot");
      });
    }
    fit();
    updateRates();
  }

  let lastRates = 0;
  function updateRates() {
    const now = performance.now() / 1000;
    lastRates = now;
    for (const [topic, [g, tspan, latched]] of rateNodes) {
      const hz = meter.rate(topic, now);
      g.classList.toggle("live", hz > 0 || latched);
      tspan.textContent = hz > 0 ? `${hz >= 10 ? hz.toFixed(0) : hz.toFixed(1)} Hz` : latched ? "latched" : "silent";
    }
  }

  context.onRender = (renderState, done) => {
    if (renderState.colorScheme) {
      for (const [k, v] of Object.entries(THEMES[renderState.colorScheme] ?? THEMES.dark)) root.style.setProperty(`--${k}`, v);
    }
    const now = performance.now() / 1000;
    let latest;
    for (const event of renderState.currentFrame ?? []) {
      if (event.topic === GRAPH_TOPIC) latest = event.message.data;
      else meter.push(event.topic, now);
    }
    if (latest !== undefined && latest !== graphText) {
      graphText = latest;
      graph = JSON.parse(latest);
      if (!graph.subsystems.some((s) => s.id === subsystem)) subsystem = graph.subsystems[0].id;
      renderTabs();
      relayout();
    }
    if (now - lastRates > 1) updateRates();
    done();
  };
  context.watch("currentFrame");
  context.watch("colorScheme");
  context.subscribe([{ topic: GRAPH_TOPIC }]);
  return () => root.remove();
}

export function activate(extensionContext) {
  extensionContext.registerPanel({ name: "Subsystem graph", initPanel });
}
