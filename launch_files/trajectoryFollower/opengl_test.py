from PyQt5.QtGui import QSurfaceFormat
from PyQt5.QtWidgets import QApplication

app = QApplication([])

fmt = QSurfaceFormat()
fmt.setVersion(3, 3)  # OpenGL version 3.3
fmt.setProfile(QSurfaceFormat.CoreProfile)
QSurfaceFormat.setDefaultFormat(fmt)
