"""Host-only adapter tests; these do not emulate or certify a ROS executor."""

import ast
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Optional


class ManualFuture:
    def __init__(self):
        self.value = None
        self.error = None
        self.finished = False

    def done(self):
        return self.finished

    def set_result(self, value):
        self.value, self.finished = value, True

    def set_exception(self, error):
        self.error, self.finished = error, True

    def cancel(self):
        self.finished = True

    def result(self):
        if self.error is not None:
            raise self.error
        return self.value

    def __await__(self):
        while not self.finished:
            yield self
        return self.result()


def adapter_classes(path: Path, **globals):
    """Execute unchanged node methods with explicit DTO/clock doubles, without ROS imports."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    body = [item for item in tree.body if isinstance(item, (ast.ClassDef, ast.FunctionDef))]
    namespace = {
        "__name__": __name__,
        "Node": object,
        "dataclass": dataclass,
        "Any": Any,
        "Optional": Optional,
        "Future": ManualFuture,
        **globals,
    }
    exec(
        compile(
            ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])), str(path), "exec"
        ),
        namespace,
    )
    return SimpleNamespace(**namespace)
