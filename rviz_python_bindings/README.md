# RVIZ Pyhon Bindings

## Usage
To use the python bindings, you need to do the following:

```python
import os
os.environ["QT_API"] = "pyside2"
from rviz import rviz_common
```

Currently only pyside2 is supported. No stubs are generated, so pylance does not work on namespaces.
The following classes are wrapped:
* VisualizationFrame
* Config
* VisualizationManager
* ViewController
* ViewManager
* DisplayGroup
* Display
* PanelDockWidget
* Tool
* ToolManager
* YamlConfigReader
* YamlConfigWriter
* Property
* BoolProperty

## Windows specifics
On windows you will need to install the following dependencies manually:

```
pip install \
    --index-url=https://download.qt.io/official_releases/QtForPython/ \
    --trusted-host download.qt.io \
    shiboken2 pyside2 shiboken2_generator
```