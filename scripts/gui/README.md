# This is a demonstration of how to use PyQt with ROS

## Dependencies

Make sure to have the following installed

-   PyQt 5
-   PyQt 5 Designer
-   PyQt 5 dev tools

## Editing the _layout_

To edit the interface, open `layout.ui` in **Qt 5 Designer**. Once all the changes are saved, run the following command

```bash
pyuic5 -x layout.ui -o layout.py
```

This will auto-generate all interface elements in Python code. The class it creates, `Ui_MainWindow` , is then imported and inherited by the main class in `main.py`

## Launching the code

Simply run `main.py`. All the logic happens in that file.
