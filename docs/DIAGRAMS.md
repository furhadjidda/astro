# Diagram Authoring Guide

All architecture diagrams live as PlantUML source files in `docs/diagrams/`.
They are rendered in Markdown files (e.g. `architecture.md`) via the public PlantUML
server — no build step, no local tooling required. The rendered SVG is fetched
at read-time by any Markdown viewer that loads external images (GitHub, GitLab, etc.).

## How the links are constructed

```
┌─────────────────────────┐   zlib deflate    ┌────────────────┐   re-alphabet   ┌──────────────────┐
│  .puml UTF-8 source     │ ──────────────────► compressed bytes│ ───────────────► encoded token     │
└─────────────────────────┘  (strip 2B header  └────────────────┘  base64 → puml  └────────┬─────────┘
                              + 4B trailer)                         alphabet                │
                                                                                            ▼
                                                         https://www.plantuml.com/plantuml/svg/<token>
```

**PlantUML alphabet** substitution maps standard base64 characters to:
`0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-_`

The server decodes the token on the fly, renders the diagram, and returns an SVG.
Nothing is stored server-side — the entire diagram is encoded in the URL.

## Workflow: editing an existing diagram

1. Edit the `.puml` file in `docs/diagrams/`.
2. Re-encode it to get a new URL:
   ```bash
   python3 docs/diagrams/encode_puml.py docs/diagrams/<name>.puml
   ```
3. Copy the printed URL and replace the old URL in the relevant Markdown file.

## Workflow: adding a new diagram

1. Create `docs/diagrams/<name>.puml` with valid PlantUML syntax.
2. Run the encoder:
   ```bash
   python3 docs/diagrams/encode_puml.py docs/diagrams/<name>.puml
   ```
3. Add an image tag to the target Markdown file:
   ```markdown
   ![Alt text](https://www.plantuml.com/plantuml/svg/<token>)

   > Source: [docs/diagrams/<name>.puml](diagrams/<name>.puml)
   ```

## Encoder script

The helper script `docs/diagrams/encode_puml.py` accepts one or more `.puml` files
and prints the full PlantUML server URL for each:

```bash
# Single file
python3 docs/diagrams/encode_puml.py docs/diagrams/tf_tree.puml

# All diagrams at once
python3 docs/diagrams/encode_puml.py docs/diagrams/*.puml
```

## PlantUML syntax quick reference

| Diagram type | Opening tag | Closing tag |
|---|---|---|
| Component / deployment | `@startuml` | `@enduml` |
| Sequence | `@startuml` | `@enduml` |
| Class | `@startuml` | `@enduml` |
| Flowchart (activity) | `@startuml` | `@enduml` |

Full reference: https://plantuml.com/

## Current diagrams

| File | Rendered in | Description |
|------|-------------|-------------|
| `docs/diagrams/system_architecture.puml` | `docs/architecture.md` | Full system component/deployment diagram |
| `docs/diagrams/data_flow.puml` | `docs/architecture.md` | ROS 2 topic data flow |
| `docs/diagrams/tf_tree.puml` | `docs/architecture.md` | ROS 2 TF transform tree |
