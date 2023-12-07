import os
import pytest


def test_import_rviz_common():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import rviz_common  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)


def test_import_visualizationframe():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import VisualizationFrame  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)


def test_import_config():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import Config  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)


def test_import_visualizationmanager():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import VisualizationManager  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)


def test_import_viewmanager():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import ViewManager  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)


def test_import_displaygroup():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import DisplayGroup  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)


def test_import_display():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import Display  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)


def test_import_paneldockwidget():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import PanelDockWidget  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)


def test_import_tool():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import Tool  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)


def test_import_toolmanager():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import ToolManager  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)


def test_import_yamlreader():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import YamlConfigReader  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)


def test_import_yamlwriter():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import YamlConfigWriter  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)


def test_import_property():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import Property  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)


def test_import_boolproperty():
    os.environ["QT_API"] = "pyside2"
    try:
        from rviz import BoolProperty  # noqa: F401
    except Exception as exc:
        pytest.fail(exc, pytrace=True)
