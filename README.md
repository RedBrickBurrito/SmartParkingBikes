# SmartParkingBikes

---------------Bicycle Parking Project for Smart City----------------

USO

Al iniciar el programa, se pide de input la posición de una bicicleta o plataforma para sacar.
Se puede insertar un valor en X no menor a 0 ni mayor a 17, y un valor Y entre 0 y 14. Al escoger
una posición en X igual o menor a 8, automáticamente se asume que se está usando la primera mitad de
la matriz de plataformas (así mismo mayor o igual a 9 toma la segunda mitad del edificio para sacar
la bicicleta).

Después se puede desplegar en consola una representación de la ruta que debe tomar la bicicleta para
salir, con la consecuencia de aumentar el tiempo. El código base, escrito en C++, está puesto de tal 
manera que se reduce al máximo el tiempo al tener comentado las partes necesarias para imprimir tal 
representación, por lo que si se quiere desplegar se requerirá modificar internamente.
