# Car + claw + Watcher: how the agents talk

Idea: every machine is a **body** with the same small interface, and every **agent** talks to bodies through
that interface (or through each other's tools). Moving the claw onto the car later changes where a body
physically is, not how anything talks.

```
              Seeed Watcher (optional "god agent")
                 |  HTTP (token)                |  HTTP (token)
   Car agent (Hermes, Pi)  <----MCP tool---->  Claw agent (friend's, MacBook)
     |  Body API :8765                              |  Body API :8766
   Car body (ROS2)                                Claw body
```

## The Body API contract (what the car already implements; the claw should copy it)
| Call | Meaning |
|---|---|
| `GET /state` | what the body sees/is doing |
| `POST /act {"behavior":..., "agent":"name", "wait":true}` | do something, optionally wait for the result |
| `POST /stop` | stop everything (always allowed) |
| `GET /events?since=N` | finished actions |
| `POST /lock {"agent","seconds"}` / `POST /unlock` | one agent (e.g. the Watcher) temporarily owns control |
| `Authorization: Bearer <token>` | required whenever the API is reachable from another machine |

Claw behaviours to agree with your friend, for example: `move_to {x,y,z}`, `grip {open|close}`, `home`,
`pick {object}`. Keep them high-level and safe-by-default: the claw's own firmware should enforce its limits.

## Who can talk to whom
- **Car mic controls the claw**: the car's Hermes gets the words, decides it is a claw task, and calls a tool
  `claw_*` (an MCP server pointing at the claw's Body API, e.g. `http://macbook.local:8766`).
  The claw agent can ask the car the same way through the car's MCP tools or Body API.
- **Watcher (later)**: calls both Body APIs; takes `/lock` when it wants to override; either body's `/stop`
  always works, so a human or any agent can halt things.
- **Claw moved onto the car**: the claw body's API now runs on the Pi (or the ESP32 next to it) on the same
  port; the agents' tools do not change, only `ROBOT_API_URL` / host.

## Safety defaults
- Bind to localhost unless another machine must call it; if reachable, a long random token is mandatory
  (the car's API refuses to start on a non-local address without one).
- All requests are clamped by the body, never trusted from the caller (agents are LLMs).
- Keep the agents' names honest (`"agent":"hermes"`): it is how locks and logs tell them apart.
- Use a private WiFi/VLAN if possible; do not expose these ports to the internet.
