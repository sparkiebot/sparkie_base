"""
Pacchetto per la connessione seriale con dispositivi esterni tramite ROS2.
"""

from .sparkie_board import *
from .sparkie_reader import SparkieDataReader

__all__ = ['SparkieDataReader']