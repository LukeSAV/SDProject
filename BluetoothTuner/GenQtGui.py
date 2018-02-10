import pysideuic

with open("Qt/untitled/mainwindow.ui", 'r') as f:
    with open("MainWindow.py", 'w')as o:
        pysideuic.compileUi(f,o , indent=0)
