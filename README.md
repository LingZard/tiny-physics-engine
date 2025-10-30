## Tiny Physics Engine (WIP)

## 🧭 Project Goals

This project aims to **build a physics engine from scratch in Rust**, primarily for **learning and exploration purposes** rather than production use.  
The goal is to **understand and implement** the fundamental components of a physics engine — including **particles, rigid bodies, constraints, collisions, impulses, and force-based solvers** — while maintaining clear and minimalistic code.

The engine will focus on **2D physics simulation**, but its architecture will be designed to be **conceptually extensible to 3D**.  
Through this project, I aim to deeply understand how modern physics engines such as **Box2D, Rapier, and MuJoCo** work internally — from mathematical models to numerical stability, from collision detection to constraint solvers.


## 🧩 Code Philosophy

> “代码应服务于理解，而非工程。”

This is a **learning-oriented project**, not an engineering-oriented one.  
The goal is **clarity and completeness of physical concepts**, not production-level optimization, modularity, or performance.

- **Favor simplicity over abstraction.**  
  Code should expose the underlying physics clearly, even if it’s less elegant from a software engineering perspective.

- **Every subsystem should be self-contained and understandable.**  
  Each module (e.g., collision detection, constraint solver, integration) should be readable as a standalone learning unit.

- **Mathematics before patterns.**  
  Prioritize physical correctness and algorithmic clarity over design patterns or generic frameworks.

- **No hidden magic.**  
  Every important numerical step (e.g., constraint solving, time integration, impulse resolution) should be explicitly written and commented.

- **Readable over reusable.**  
  The main purpose of the codebase is to learn, not to reuse — verbosity is acceptable if it reveals the underlying logic.
