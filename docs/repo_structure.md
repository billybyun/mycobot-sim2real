
# repo_structure.md — Repository Layout

Recommended structure for compatibility with Cursor/Claude.

mycobot-vla/

├── docs/
│   ├── MASTER.md
│   ├── plan.md
│   ├── architecture.md
│   ├── hardware.md
│   ├── roadmap.md
│   └── repo_structure.md
│
├── src/
│   └── mycobot_vla/
│       ├── robot/
│       ├── vision/
│       ├── planning/
│       ├── vla/
│       ├── logging/
│       └── sim/
│
├── scripts/
├── data/
└── tests/

Rules:

• scripts should contain minimal logic
• reusable code belongs in src/mycobot_vla
• avoid hardcoding device paths
