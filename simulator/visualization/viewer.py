"""Pygame viewer: arena, robot, sensor spots, the ROBOT'S OWN map (nodes,
edges, planned route) overlaid on the true arena, clocks, score and the
navigation log.

Keys: SPACE pause/resume, +/- simulation speed, M toggle robot map,
      T toggle trail, Q / Esc quit.
"""
from __future__ import annotations

import math

import pygame

from ..arena import Arena
from ..scoring.match import Match

BG = (14, 14, 18)
LINE = (235, 235, 235)
BOX = (250, 250, 250)
CP = (60, 220, 60)
ROBOT = (70, 150, 255)
SENSOR_ON = (255, 210, 40)
SENSOR_OFF = (90, 90, 90)
MAP_NODE = (255, 120, 60)
MAP_EDGE = (255, 150, 90)
ROUTE = (255, 60, 200)
TEXT = (220, 220, 220)


class Viewer:
    def __init__(self, match: Match, size: int = 820, speed: float = 4.0):
        pygame.init()
        self.m = match
        self.a: Arena = match.arena
        self.size = size
        self.panel = 380
        self.screen = pygame.display.set_mode((size + self.panel, size))
        pygame.display.set_caption(f"Meshmerize simulator - {self.a.name}")
        self.font = pygame.font.SysFont("consolas", 14)
        self.big = pygame.font.SysFont("consolas", 18, bold=True)
        self.speed = speed
        self.paused = False
        self.show_map = True
        self.show_trail = True
        self.trail: list[tuple[int, int]] = []
        self.clock = pygame.time.Clock()
        self.sim_ms_shown = 0
        self.quit = False
        self.scale = size / self.a.size[0]

    # coordinate transforms -------------------------------------------------
    def px(self, x: float, y: float) -> tuple[int, int]:
        return int(x * self.scale), int(self.size - y * self.scale)

    def map_to_world(self, mx: float, my: float) -> tuple[float, float]:
        sx, sy, sh = self.a.start
        th = math.radians(sh)
        nx, ny = math.cos(th), math.sin(th)    # robot map "north" = start heading
        ex, ey = math.sin(th), -math.cos(th)   # map "east" = right of start heading
        return sx + mx * ex + my * nx, sy + mx * ey + my * ny

    # drawing -------------------------------------------------------------------
    def draw_arena(self):
        s = self.screen
        s.fill(BG)
        pygame.draw.rect(s, (60, 60, 70), (0, 0, self.size, self.size), 1)
        w = max(2, int(self.a.line_width * self.scale))
        for x1, y1, x2, y2 in self.a.segments:
            pygame.draw.line(s, LINE, self.px(x1, y1), self.px(x2, y2), w)
        bx, by, bw, bh = self.a.end_box
        p = self.px(bx, by + bh)
        pygame.draw.rect(s, BOX, (p[0], p[1], int(bw * self.scale), int(bh * self.scale)))
        for i, (cx, cy) in enumerate(self.a.checkpoints):
            col = (40, 160, 40) if i in self.m.crossed else CP
            pygame.draw.circle(s, col, self.px(cx, cy), int(60 * self.scale), 2)
        sx, sy, _ = self.a.start
        s.blit(self.font.render("START", True, CP), self.px(sx + 30, sy + 40))

    def edge_polyline(self, nodes, e, reverse=False):
        """Map-frame polyline of an edge from its recorded segments and corners."""
        dxy = {0: (0, 1), 1: (1, 0), 2: (0, -1), 3: (-1, 0)}
        segs = e["segs"]
        if not reverse:
            x, y = nodes[e["a"]][0], nodes[e["a"]][1]
            h = e["da"]
            order = [(l, t) for l, t in segs]
        else:
            x, y = nodes[e["b"]][0], nodes[e["b"]][1]
            h = e["db"]
            lens = [l for l, _ in segs][::-1]
            turns = [-t for _, t in segs[:-1]][::-1] + [0]
            order = list(zip(lens, turns))
        pts = [(x, y)]
        for l, t in order:
            x += dxy[h][0] * l
            y += dxy[h][1] * l
            pts.append((x, y))
            h = (h - t) % 4
        return pts

    def draw_robot_map(self, snap: dict):
        if not self.show_map:
            return
        nodes = snap["nodes"]
        edges = snap["edges"]
        for e in edges:
            pts = [self.px(*self.map_to_world(*q)) for q in self.edge_polyline(nodes, e)]
            pygame.draw.lines(self.screen, MAP_EDGE, False, pts, 1)
        if snap["route"]["valid"]:
            for node, _exit, ei in snap["route"]["steps"]:
                e = edges[ei]
                rev = not (e["a"] == node and e["da"] == _exit)
                pts = [self.px(*self.map_to_world(*q)) for q in self.edge_polyline(nodes, e, rev)]
                pygame.draw.lines(self.screen, ROUTE, False, pts, 3)
        for i, n in enumerate(nodes):
            col = {0: MAP_NODE, 1: (200, 80, 80), 2: CP, 3: (255, 255, 0)}.get(n[3], MAP_NODE)
            p = self.px(*self.map_to_world(n[0], n[1]))
            pygame.draw.circle(self.screen, col, p, 5)
            self.screen.blit(self.font.render(str(i), True, col), (p[0] + 6, p[1] - 6))

    def draw_robot(self):
        sim = self.m.sim
        x, y, th = sim.pose
        p = sim.p
        c, s = math.cos(th), math.sin(th)
        corners = []
        for fx, fy in ((p.body_front_mm, 50), (p.body_front_mm, -50), (-p.body_rear_mm, -50), (-p.body_rear_mm, 50)):
            corners.append(self.px(x + fx * c - fy * s, y + fx * s + fy * c))
        pygame.draw.polygon(self.screen, ROBOT, corners, 2)
        pygame.draw.circle(self.screen, ROBOT, self.px(x, y), 3)
        px_, py_ = sim.sensors.sensor_points(x, y, th)
        core_feat = sim.raw
        for i in range(len(px_)):
            on = core_feat[i] < (p.raw_black + p.raw_white) / 2
            pygame.draw.circle(self.screen, SENSOR_ON if on else SENSOR_OFF, self.px(px_[i], py_[i]), 2)
        if sim.led:
            pygame.draw.circle(self.screen, (255, 30, 30), self.px(x - p.body_rear_mm * c, y - p.body_rear_mm * s), 6)
        if self.show_trail:
            self.trail.append(self.px(x, y))
            if len(self.trail) > 1:
                pygame.draw.lines(self.screen, (60, 90, 160), False, self.trail[-4000:], 1)

    def draw_panel(self, snap: dict):
        x0 = self.size + 10
        y = 10
        sim = self.m.sim

        def put(t, f=None, col=TEXT):
            nonlocal y
            self.screen.blit((f or self.font).render(t, True, col), (x0, y))
            y += 20 if f else 17

        put("MESHMERIZE SIM", self.big)
        put(f"phase  : {self.m.phase}")
        put(f"state  : {sim.core.state}  stage {sim.core.stage}")
        put(f"time   : {sim.t_us / 1e6:7.2f} s   speed x{self.speed:g}")
        put(f"LED    : {'ON' if sim.led else 'off'}   fault {sim.core.fault}")
        put(f"checkpoints crossed : {len(self.m.crossed)}/{len(self.a.checkpoints)}")
        put(f"restarts used       : {self.m.outcome.restarts}/3")
        put(f"map nodes/edges     : {len(snap['nodes'])}/{len(snap['edges'])}")
        r = snap["route"]
        put(f"route  : {'%.0f mm, %d steps' % (r['length'], len(r['steps'])) if r['valid'] else '-'}")
        put(f"true shortest       : {self.m.true_shortest:.0f} mm")
        put(f"battery {sim.body.vbat:.2f} V  {sim.body.current:.2f} A")
        y += 8
        put("log", self.big)
        for line in sim.log[-26:]:
            put(line[:46])
        y = self.size - 40
        put("SPACE pause  +/- speed  M map  T trail  Q quit", col=(140, 140, 140))

    # main hook -----------------------------------------------------------------
    def __call__(self, match: Match):
        if self.quit:
            raise KeyboardInterrupt
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                self.quit = True
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                    self.quit = True
                elif ev.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    self.speed = min(64.0, self.speed * 2)
                elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    self.speed = max(0.25, self.speed / 2)
                elif ev.key == pygame.K_m:
                    self.show_map = not self.show_map
                elif ev.key == pygame.K_t:
                    self.show_trail = not self.show_trail
        while self.paused and not self.quit:
            for ev in pygame.event.get():
                if ev.type == pygame.KEYDOWN and ev.key == pygame.K_SPACE:
                    self.paused = False
                elif ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key in (pygame.K_q, pygame.K_ESCAPE)):
                    self.quit = True
            self.clock.tick(30)
        sim_ms = match.sim.t_us // 1000
        frame_every = max(1, int(16 * self.speed))
        if sim_ms - self.sim_ms_shown < frame_every:
            return
        self.sim_ms_shown = sim_ms
        snap = match.sim.core.export()
        self.draw_arena()
        self.draw_robot_map(snap)
        self.draw_robot()
        self.draw_panel(snap)
        pygame.display.flip()
        self.clock.tick(60)
