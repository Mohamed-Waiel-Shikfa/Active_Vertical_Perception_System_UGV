import assert from "node:assert/strict";
import test from "node:test";

import ELK from "elkjs/lib/elk.bundled.js";

import { RateMeter, edgePath, elkGraph, shortName, view } from "./graph.js";

const graph = {
  namespace: "j100_0000",
  subsystems: [{ id: "drive", label: "Drive" }, { id: "lift", label: "Lift" }],
  topics: [
    { name: "/j100_0000/platform/cmd_vel", type: "geometry_msgs/msg/TwistStamped",
      publishers: ["/j100_0000/twist_mux"], subscribers: ["/j100_0000/platform_velocity_controller"], subsystems: ["drive"] },
    { name: "/j100_0000/greenhouse/lift/velocity", type: "std_msgs/msg/Float64",
      publishers: [], subscribers: ["/greenhouse_sensor_bridge"], subsystems: ["lift"] },
  ],
};

test("shortName replaces only the robot namespace", () => {
  assert.equal(shortName("/j100_0000/platform/cmd_vel", "j100_0000"), "~/platform/cmd_vel");
  assert.equal(shortName("/tf", "j100_0000"), "/tf");
  assert.equal(shortName("/j100_00001/x", "j100_0000"), "/j100_00001/x");
  assert.equal(shortName("/tf", ""), "/tf");
});

test("view keeps one subsystem and links publisher -> topic -> subscriber", () => {
  const v = view(graph, "drive");
  assert.deepEqual(v.nodes, ["/j100_0000/platform_velocity_controller", "/j100_0000/twist_mux"]);
  assert.deepEqual(v.edges.map((e) => [e.source, e.target]), [
    ["n:/j100_0000/twist_mux", "t:/j100_0000/platform/cmd_vel"],
    ["t:/j100_0000/platform/cmd_vel", "n:/j100_0000/platform_velocity_controller"],
  ]);
});

test("ELK lays the view out left to right with orthogonal edges", async () => {
  const laid = await new ELK().layout(elkGraph(view(graph, "drive"), graph.namespace, (s) => s.length * 7));
  const x = Object.fromEntries(laid.children.map((c) => [c.id, c.x]));
  assert.ok(x["n:/j100_0000/twist_mux"] < x["t:/j100_0000/platform/cmd_vel"]);
  assert.ok(x["t:/j100_0000/platform/cmd_vel"] < x["n:/j100_0000/platform_velocity_controller"]);
  for (const e of laid.edges) {
    const s = e.sections[0];
    const pts = [s.startPoint, ...(s.bendPoints ?? []), s.endPoint];
    for (let i = 1; i < pts.length; i++) assert.ok(pts[i].x === pts[i - 1].x || pts[i].y === pts[i - 1].y);
  }
});

test("edgePath rounds a corner and ends on the end point", () => {
  const d = edgePath({ startPoint: { x: 0, y: 0 }, bendPoints: [{ x: 20, y: 0 }], endPoint: { x: 20, y: 20 } });
  assert.equal(d, "M0,0 L14,0 Q20,0 20,6 L20,20");
});

test("RateMeter counts messages inside the window", () => {
  const m = new RateMeter();
  for (let i = 0; i < 30; i++) m.push("/a", 10 + i * 0.1);
  assert.equal(m.rate("/a", 12.95), 10);
  assert.equal(m.rate("/a", 100), 0);
  assert.equal(m.rate("/missing", 1), 0);
});

test("view hides unpublished inputs unless asked", () => {
  assert.equal(view(graph, "lift").topics.length, 0);
  assert.equal(view(graph, "lift", true).topics.length, 1);
});
