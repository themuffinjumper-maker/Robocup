import json
from pathlib import Path

Import("env")

project_dir = Path(env.subst("$PROJECT_DIR"))
source_path = project_dir / "debug_config.json"
output_path = project_dir / "include" / "debug_config.generated.h"
local_source_path = project_dir / "local_mission.json"
local_output_path = project_dir / "include" / "local_mission.generated.h"

# Fail the build early with a useful location if somebody edits invalid JSON.
config = json.loads(source_path.read_text(encoding="utf-8"))
compact = json.dumps(config, separators=(",", ":"), ensure_ascii=True)

generated = (
    "#pragma once\n\n"
    "namespace EmbeddedDebugConfig {\n"
    f'inline constexpr char JSON[] = R"DEBUG_JSON({compact})DEBUG_JSON";\n'
    "}\n"
)

if not output_path.exists() or output_path.read_text(encoding="utf-8") != generated:
    output_path.write_text(generated, encoding="utf-8")

# The arena planner exports one JSON file. Embed the exact export into flash so
# the Teensy can load the same route, rotated features, sensor poses, and nav
# tuning without relying on Bluetooth at run time.
if local_source_path.exists():
    local_config = json.loads(local_source_path.read_text(encoding="utf-8"))
    if not isinstance(local_config, dict):
        raise ValueError(f"{local_source_path} must contain a JSON object")
    local_enabled = local_config.get("enabled") is True
    if local_enabled and not all(key in local_config for key in ("map", "mission", "parameters")):
        raise ValueError(
            f"{local_source_path} is enabled but must contain map, mission, and parameters"
        )
    local_compact = json.dumps(local_config, separators=(",", ":"), ensure_ascii=True)
else:
    local_enabled = False
    local_compact = "{}"

local_generated = (
    "#pragma once\n\n"
    "namespace LocalMissionConfig {\n"
    f"inline constexpr bool ENABLED = {'true' if local_enabled else 'false'};\n"
    f"inline constexpr char JSON[] = {json.dumps(local_compact)};\n"
    "}\n"
)

if not local_output_path.exists() or local_output_path.read_text(encoding="utf-8") != local_generated:
    local_output_path.write_text(local_generated, encoding="utf-8")
