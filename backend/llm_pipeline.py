import json
import re
from typing import Dict, List, Tuple

from .safety import validate_waypoints

try:
    from jsonschema import Draft202012Validator

    JSONSCHEMA_AVAILABLE = True
except Exception:
    Draft202012Validator = None
    JSONSCHEMA_AVAILABLE = False


MISSION_SCHEMA = {
    "type": "object",
    "required": ["mission_type", "waypoints"],
    "properties": {
        "mission_type": {"type": "string"},
        "waypoints": {
            "type": "array",
            "minItems": 1,
            "items": {
                "type": "object",
                "required": ["lat", "lon", "alt"],
                "properties": {
                    "lat": {"type": "number"},
                    "lon": {"type": "number"},
                    "alt": {"type": "number"},
                    "speed": {"type": "number"},
                    "hold_s": {"type": "number"},
                    "action": {
                        "type": "string",
                        "enum": [
                            "waypoint",
                            "takeoff",
                            "land",
                            "land_and_hold",
                            "loiter",
                            "takeoff_after_hold",
                            "descend",
                            "goto_low_alt",
                            "rtl",
                            "disarm",
                        ],
                    },
                },
            },
        },
        "constraints": {"type": "object"},
    },
}


PROMPT_TEMPLATE = """You are a UAV mission planning assistant.
Return ONLY valid JSON with keys: mission_type, waypoints, constraints.
Safety rules: altitude between 5 and 120 meters, no more than 200 waypoints.
Waypoints may include optional fields: action, speed, hold_s.
Use action values such as waypoint, takeoff, land, land_and_hold, loiter, takeoff_after_hold, descend, goto_low_alt, rtl, disarm.
User request: {user_request}
"""


class LLMMissionPipeline:
    def __init__(self, max_retries: int = 2):
        self.max_retries = max_retries

    def build_prompt(self, user_request: str) -> str:
        return PROMPT_TEMPLATE.format(user_request=user_request.strip())

    def _build_retry_prompt(self, user_request: str, errors: List[str]) -> str:
        return (
            self.build_prompt(user_request)
            + "\nPrevious output failed validation with errors:\n"
            + "\n".join(f"- {e}" for e in errors[:8])
            + "\nReturn corrected JSON only."
        )

    def _is_prompt_injection_or_unsafe(self, user_request: str) -> bool:
        patterns = [
            r"ignore\s+previous\s+instructions",
            r"reveal\s+system\s+prompt",
            r"disable\s+safety",
            r"bypass\s+validation",
            r"restricted\s+airspace",
        ]
        lowered = user_request.lower()
        return any(re.search(p, lowered) for p in patterns)

    def _extract_json_object(self, content: str) -> str:
        content = content.strip()
        if content.startswith("{") and content.endswith("}"):
            return content

        start = content.find("{")
        end = content.rfind("}")
        if start >= 0 and end > start:
            return content[start : end + 1]
        return content

    def _validate_schema(self, mission: Dict) -> List[str]:
        if JSONSCHEMA_AVAILABLE:
            validator = Draft202012Validator(MISSION_SCHEMA)
            return [f"Schema validation error: {e.message}" for e in validator.iter_errors(mission)]

        errors: List[str] = []
        if not isinstance(mission, dict):
            errors.append("Mission payload must be a JSON object.")
            return errors

        if not isinstance(mission.get("mission_type"), str):
            errors.append("mission_type must be a string.")

        waypoints = mission.get("waypoints")
        if not isinstance(waypoints, list) or len(waypoints) < 1:
            errors.append("waypoints must be a non-empty list.")
        else:
            for i, wp in enumerate(waypoints):
                if not isinstance(wp, dict):
                    errors.append(f"WP{i + 1} must be an object.")
                    continue
                for key in ("lat", "lon", "alt"):
                    if key not in wp:
                        errors.append(f"WP{i + 1} missing {key}.")
                    elif not isinstance(wp[key], (int, float)):
                        errors.append(f"WP{i + 1} {key} must be numeric.")

        return errors

    def parse_and_validate_json(self, content: str) -> Tuple[bool, Dict, List[str]]:
        errors: List[str] = []
        payload = self._extract_json_object(content)
        try:
            mission = json.loads(payload)
        except json.JSONDecodeError as exc:
            return False, {}, [f"JSON parse error: {exc}"]

        errors.extend(self._validate_schema(mission))

        if errors:
            return False, mission, errors

        valid, constraint_errors = validate_waypoints(mission["waypoints"])
        if not valid:
            errors.extend(constraint_errors)

        return len(errors) == 0, mission, errors

    def generate_with_retries(self, user_request: str, llm_call) -> Dict:
        if self._is_prompt_injection_or_unsafe(user_request):
            return {
                "ok": False,
                "mission": {},
                "errors": ["Mission request rejected: unsafe or adversarial prompt content."],
                "requires_human_approval": True,
            }

        prompt = self.build_prompt(user_request)
        all_errors: List[str] = []

        for _ in range(self.max_retries + 1):
            response_text = llm_call(prompt)
            ok, mission, errors = self.parse_and_validate_json(response_text)
            if ok:
                return {
                    "ok": True,
                    "mission": mission,
                    "errors": [],
                    "requires_human_approval": True,
                }
            all_errors.extend(errors)
            prompt = self._build_retry_prompt(user_request, errors)

        return {
            "ok": False,
            "mission": {},
            "errors": all_errors,
            "requires_human_approval": True,
        }
