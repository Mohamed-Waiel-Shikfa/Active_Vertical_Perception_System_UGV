// Pure helpers for the panel: subsystem view of the graph message, ELK input, message rates.

export const GRAPH_TOPIC = "/greenhouse/viz/graph";
export const RATE_WINDOW_S = 3;

// "/j100_0000/platform/cmd_vel" -> "~/platform/cmd_vel"; names outside the namespace stay as they are.
export function shortName(name, namespace) {
  const prefix = namespace ? `/${namespace}/` : "";
  return prefix && name.startsWith(prefix) ? `~/${name.slice(prefix.length)}` : name;
}

export function shortType(type) {
  return type.split("/").pop();
}

// One subsystem of the graph message: ROS nodes, topics and node->topic->node edges.
// Topics nobody publishes (unused inputs such as e-stop or RC teleop) only with showUnused.
export function view(graph, subsystem, showUnused = false) {
  const topics = graph.topics.filter((t) => t.subsystems.includes(subsystem) && (showUnused || t.publishers.length));
  const nodes = [...new Set(topics.flatMap((t) => [...t.publishers, ...t.subscribers]))].sort();
  const edges = topics.flatMap((t) => [
    ...t.publishers.map((n) => ({ id: `p ${n} ${t.name}`, source: `n:${n}`, target: `t:${t.name}` })),
    ...t.subscribers.map((n) => ({ id: `s ${t.name} ${n}`, source: `t:${t.name}`, target: `n:${n}` })),
  ]);
  return { nodes, topics, edges };
}

// Same view, same layout: a key that changes only when nodes, topics or edges do.
export function viewKey(v) {
  return JSON.stringify([v.nodes, v.edges.map((e) => e.id)]);
}

export const NODE_HEIGHT = 30;
export const TOPIC_HEIGHT = 36;
const PAD = 14;

// ELK layered graph (direction RIGHT or DOWN), orthogonal edges. measure(text, mono) -> width in px.
export function elkGraph(v, namespace, measure, direction = "RIGHT") {
  const children = [
    ...v.nodes.map((n) => ({ id: `n:${n}`, width: measure(shortName(n, namespace), false) + 2 * PAD, height: NODE_HEIGHT })),
    ...v.topics.map((t) => ({
      id: `t:${t.name}`,
      width: Math.max(measure(shortName(t.name, namespace), true), measure(`${shortType(t.type)}  00.0 Hz`, false)) + 2 * PAD,
      height: TOPIC_HEIGHT,
    })),
  ];
  return {
    id: "root",
    layoutOptions: {
      "elk.algorithm": "layered",
      "elk.direction": direction,
      "elk.edgeRouting": "ORTHOGONAL",
      "elk.layered.spacing.nodeNodeBetweenLayers": "56",
      "elk.spacing.nodeNode": "18",
      "elk.spacing.edgeNode": "14",
      "elk.spacing.edgeEdge": "8",
      "elk.layered.nodePlacement.strategy": "NETWORK_SIMPLEX",
      "elk.layered.considerModelOrder.strategy": "NODES_AND_EDGES",
      "elk.padding": "[top=24,left=24,bottom=24,right=24]",
    },
    children,
    edges: v.edges.map((e) => ({ id: e.id, sources: [e.source], targets: [e.target] })),
  };
}

// SVG path through an ELK edge section, corners rounded by up to r px.
export function edgePath(section, r = 6) {
  const pts = [section.startPoint, ...(section.bendPoints ?? []), section.endPoint];
  let d = `M${pts[0].x},${pts[0].y}`;
  for (let i = 1; i < pts.length - 1; i++) {
    const [a, b, c] = [pts[i - 1], pts[i], pts[i + 1]];
    const k = Math.min(r, Math.hypot(b.x - a.x, b.y - a.y) / 2, Math.hypot(c.x - b.x, c.y - b.y) / 2);
    const toward = (p, q) => ({ x: p.x + Math.sign(q.x - p.x) * k, y: p.y + Math.sign(q.y - p.y) * k });
    const p1 = toward(b, a);
    const p2 = toward(b, c);
    d += ` L${p1.x},${p1.y} Q${b.x},${b.y} ${p2.x},${p2.y}`;
  }
  const end = pts[pts.length - 1];
  return `${d} L${end.x},${end.y}`;
}

// Messages per second per topic over the last RATE_WINDOW_S of receive time (s).
export class RateMeter {
  #times = new Map();

  push(topic, t) {
    let list = this.#times.get(topic);
    if (!list) this.#times.set(topic, (list = []));
    list.push(t);
  }

  rate(topic, now) {
    const list = this.#times.get(topic);
    if (!list) return 0;
    while (list.length && list[0] < now - RATE_WINDOW_S) list.shift();
    return list.length / RATE_WINDOW_S;
  }
}
