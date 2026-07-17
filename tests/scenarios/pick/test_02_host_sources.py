import ast
import unittest
from pathlib import Path


class TestHostAllowedSources(unittest.TestCase):
    def test_lji_and_servo_listed_in_host_allowlist(self) -> None:
        root = next(
            p
            for p in Path(__file__).resolve().parents
            if (p / "apps" / "host" / "main.py").exists()
        )
        host_py = root / "apps" / "host" / "main.py"
        tree = ast.parse(host_py.read_text(encoding="utf-8"))
        allowed: set[str] = set()
        for node in ast.walk(tree):
            if not isinstance(node, ast.FunctionDef) or node.name != "_is_allowed_source":
                continue
            for child in ast.walk(node):
                if isinstance(child, ast.Tuple):
                    for elt in child.elts:
                        if isinstance(elt, ast.Constant) and isinstance(elt.value, str):
                            allowed.add(str(elt.value))
        self.assertIn("lji", allowed)
        self.assertIn("lji_step", allowed)
        self.assertIn("servo", allowed)
        self.assertIn("ik", allowed)
        self.assertIn("experiment", allowed)


if __name__ == "__main__":
    unittest.main()
