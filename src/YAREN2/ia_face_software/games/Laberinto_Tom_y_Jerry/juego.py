#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pygame
from pygame.locals import *
import time
import mapa
import sys
import os 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String


current_dir = os.path.abspath(os.path.join(os.path.dirname(__file__))) +"/"

pygame.mixer.pre_init(44100,16,2,1024)
pygame.init()



BLANCO = (255,255,255)
AMARILLO = (255,255,0)

tipoLetra = pygame.font.Font(current_dir+'Grandezza.ttf', 30)
tipoLetra2 = pygame.font.Font(current_dir+'Grandezza.ttf', 35)

imagenDeFondo = current_dir+'Noticia_TomJerry.jpg'



imagenGatoContento = current_dir+'gato.png'

imagenRatonContento = current_dir+'raton1.png'

imagenQueso = current_dir+'q.png'



visor = pygame.display.set_mode((800, 480))


def pausa():
   # Esta función hace que se espera hasta que se pulse una tecla
   esperar = True
   while esperar:
      for evento in pygame.event.get():
          if evento.type == KEYDOWN:
              esperar = False
  

def mostrarIntro():
   # Muestra la pantalla de inicio y espera
   fondo = pygame.image.load(imagenDeFondo).convert()
   visor.blit(fondo, (0,0))
   mensaje = 'Pulsa una tecla para comenzar'
   texto = tipoLetra.render(mensaje, True, AMARILLO)
  
   visor.blit(texto, (60,500,200,30)) #60,550,300,30
   pygame.display.update()
   pausa()


# La clase Jugador implementa el sprite del jugador. En este ejemplo es algo
# muy sencillo, simplemente un gato que se mueve a izquierda, derecha,
# arriba y abajo con las teclas o, p, q y a.


def seleccionar_pantalla():
    pygame.init()
    pantallas = pygame.display.get_desktop_sizes()
    print("Detectando pantallas disponibles:")
    for i, res in enumerate(pantallas):
        print(f"Pantalla {i+1}: {res[0]}x{res[1]}")

    seleccionada = 0
    seleccion_hecha = False
    fuente = pygame.font.SysFont("Arial", 30)
    window_temp = pygame.display.set_mode((500, 300))
    pygame.display.set_caption("Seleccionar Pantalla")

    while not seleccion_hecha:
        window_temp.fill((0, 0, 0))
        texto = fuente.render(f"Seleccione la pantalla (1-{len(pantallas)}): {seleccionada+1}", True, (255, 255, 255))
        window_temp.blit(texto, (50, 100))
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RIGHT:
                    seleccionada = (seleccionada + 1) % len(pantallas)
                elif event.key == pygame.K_LEFT:
                    seleccionada = (seleccionada - 1) % len(pantallas)
                elif event.key == pygame.K_RETURN:
                    seleccion_hecha = True

    # Calcular la posición global de la pantalla seleccionada
    x_global = sum(pantallas[i][0] for i in range(seleccionada))
    y_global = 0  # Asumiendo que las pantallas están alineadas horizontalmente

    pygame.display.quit()
    return seleccionada, pantallas[seleccionada], (x_global, y_global)

def centrar_ventana(pantalla_res, position):
    global window_position
    x_global, y_global = position
    window_width, window_height = 800, 480  # Dimensiones de la ventana
    screen_width, screen_height = pantalla_res
    window_position = [
        (screen_width - window_width) // 2 + x_global,
        (screen_height - window_height) // 2 + y_global
    ]



class imagenRatonContento_class( pygame.sprite.Sprite ):

    def __init__( self, posX, posY ):
        pygame.sprite.Sprite.__init__( self )
        self.image = pygame.image.load(current_dir+'raton1.png').convert()
        self.image.set_colorkey((255,255,255))
        self.rect = self.image.get_rect()
            
        # Aprovechando el constructor de la clase, situamos la posición inicial
        # del sprite en las coordenadas posX y posY        
        
        
        self.rect.topleft = (posX, posY)
            
        
        # dy y dx son las velocidades verticales del sprite. Inicialmente son 0.       
        
        self.dy = 0
        self.dx = 0


    def update(self):
                
        # Cuando se mueve el sprite se usa esta función. Pero ahora hay que ir con
        # cuidado. Si se mueve el spritey hay colisión con la pared, en realidad
        # no se debería poder mover. Así que necesitamos una manera de deshacer
        # el movimiento y que no se muestre realmente en pantalla. ¿Cómo hacerlo?
        # Lo que hacemos es almacenar en la variable pos la posición del sprite
        # antes de que se mueva, así para deshacer el movimiento sólo hay que
        # volver a colocar el sprite en donde indica pos.        
        
        
        
        
        self.pos = self.rect.topleft
    
        # Una vez hecho eso, ya podemos hacer la tentativa de mover el sprite.        
        
        
        self.rect.move_ip(self.dx,self.dy)
        
    def deshacer(self):

        # Ésta es la función en la que deshacemos el movimiento, si hace falta.
        # Como hemos dicho, ponemos el srpite donde estaba antes, en pos.        
        
        
        self.rect.topleft = self.pos



class imagenGatoContento_class( pygame.sprite.Sprite ):

    def __init__( self, posX, posY ):
        pygame.sprite.Sprite.__init__( self )
        self.image = pygame.image.load(current_dir+'gato.png').convert()
        self.image.set_colorkey((255,255,255))
        self.rect = self.image.get_rect()
            
        self.rect.topleft = (posX, posY)
            
        self.dy = 0
        self.dx = 0
                
    def update(self):
                
        self.pos = self.rect.topleft
    
        self.rect.move_ip(self.dx,self.dy)
        
    def deshacer(self):

        self.rect.topleft = self.pos

class juego:
    def __init__(self):
        rospy.init_node('juegoTomyJerry')
        self.r = rospy.Rate(10)
        self.position_vertical_Jerry=0
        self.position_horizontal_Jerry=0
        self.position_vertical_Tom=0
        self.position_horizontal_Tom=0
        
        self.sub_commands = rospy.Subscriber('/joint_information', JointState, self.callback_commands)
        self.window_width, self.window_height = 800, 480
        
    
            
    def callback_commands(self,msg):
        try:
            list_position = msg.position
            self.position_vertical_Jerry = list_position[11]
            self.position_horizontal_Jerry= list_position[10]
            self.position_vertical_Tom = list_position[7]
            self.position_horizontal_Tom= list_position[6]
            print(f"Recibido: {self.position_vertical_Jerry}")
            #rospy.loginfo(f"Received joint states: {data.position}")
        except ValueError:
            rospy.logerr("Error al procesar el mensaje recibido.")


    # Empezamos el programa de manera efectiva, por fin. Primero definimos la
    # ventana de 800x600 en la que se va a desarrollar.

    def loop(self):
        #self.handle_client_connection()        
        self.main()

    def main(self):
        while not rospy.is_shutdown():
            
            # Configuración de la ventana
            #pygame.init()

            

            
            


            #print(self.position_vertical_Jerry)
            pygame.mouse.set_visible(False)
            mostrarIntro()
            time.sleep(0.75)
            os.environ['SDL_VIDEO_WINDOW_POS'] = "0,0"
            os.environ['SDL_VIDEO_CENTERED'] = "0"  # Asegura que no intente centrarla automáticamente

            # Selección de pantalla
            seleccionada, res_pantalla,position = seleccionar_pantalla()
            centrar_ventana(res_pantalla,position)
            # Configurar ventana en la posición seleccionada
            os.environ['SDL_VIDEO_WINDOW_POS'] = f"{window_position[0]},{window_position[1]}"
            visor = pygame.display.set_mode((self.window_width, self.window_height), pygame.NOFRAME)
            pygame.display.set_caption("Ejemplo de Mapa")
            #visor = pygame.display.set_mode((800, 480), pygame.NOFRAME | pygame.FULLSCREEN)
            #pygame.display.set_caption('Ejemplo de Mapa')

            # Ahora creamos el sprite del jugador en la posición (50,200)...


            imagenRatonContento = imagenRatonContento_class(50,390) #50,200
            imagenGatoContento = imagenGatoContento_class(600,120)  #50,300

            grupoimagenRatonContento = pygame.sprite.RenderUpdates( imagenRatonContento )
            grupoimagenGatoContento = pygame.sprite.RenderUpdates( imagenGatoContento )

            nivel = mapa.Mapa(current_dir+'mapa.txt')

            # Como es habitual, creamos el reloj que controlará la velocidad de la animación.   

            reloj = pygame.time.Clock()


            # Empezamos el bucle de la animación del juego. Recuerda que cada ciclo de este
            # bucle representa un fotograma de la animación del juego.




            while True:
                
                # Ponemos el reloj a 60 fotogramas por segundo.   
                
                reloj.tick(60)
                
                # Miramos en la lista de eventos si se cierra la ventana o se pulsa la tecla
                # escape, en cuyo caso hay que terminar el programa.
                
                for evento in pygame.event.get():
                    if evento.type == QUIT or (evento.type == KEYDOWN and evento.key == K_ESCAPE):
                        pygame.quit()
                        sys.exit()
                
                # En el plano del nivel hemos dejado una salida que deja salir de la pantalla.
                # Supuestamente esa sería la forma de cambiar de nivel. En el ejemplo que
                # estamos escribiendo, simplemente también se acaba el juego. Si se quisiera
                # hacer otra cosa, éste sería el lugar.
                
                if imagenGatoContento.rect.right > 800:
                    pygame.quit()
                    sys.exit()
                
                # Ha llegado el momento de mover el sprite. Para eso, primero miramos
                # la lista de teclas que se han pulsado...
                
                teclasPulsadas = pygame.key.get_pressed()
                
                # ... y según cuales sean éstas, hacemos el desplazamiento correspondiente
                # en la dirección adecuada modificando su velocidad.
                
                if teclasPulsadas[K_a]:
                    imagenGatoContento.dx = -2
                elif teclasPulsadas[K_d]:
                    imagenGatoContento.dx = 2
                else:
                    imagenGatoContento.dx = 0
                    
                if teclasPulsadas[K_w]:
                    imagenGatoContento.dy = -2
                elif teclasPulsadas[K_x]:
                    imagenGatoContento.dy = 2
                else:
                    imagenGatoContento.dy = 0
                

                if 0.1 <= self.position_horizontal_Tom <= 2:
                    imagenGatoContento.dx = -2
                elif -2 <= self.position_horizontal_Tom <= -0.1:
                    imagenGatoContento.dx = 2
                else:
                    imagenGatoContento.dx = 0
                    
                if -0.1 <= self.position_vertical_Tom <= 2:
                    imagenGatoContento.dy = -2
                elif -2 <= self.position_vertical_Tom <= -0.3:
                    imagenGatoContento.dy = 2
                else:
                    imagenGatoContento.dy = 0
                
                
                
                # Modificada su velocidad, ya se puede llamar a la función que cambia su
                # posición de acuerdo con ello...    
                
                
                
                
                if imagenRatonContento.rect.right > 800:
                    pygame.quit()
                    sys.exit()
                
            
                
                teclasPulsadas = pygame.key.get_pressed()
                

                
                if teclasPulsadas[K_LEFT]:
                    imagenRatonContento.dx = -2
                    print(self.position_vertical_Jerry)
                elif teclasPulsadas[K_RIGHT]:
                    imagenRatonContento.dx = 2
                else:
                    imagenRatonContento.dx = 0
             

                if teclasPulsadas[K_UP]:
                    imagenRatonContento.dy = -2
                elif teclasPulsadas[K_DOWN]:
                    imagenRatonContento.dy = 2
                else:
                    imagenRatonContento.dy = 0
                

                if -0.5 < self.position_horizontal_Jerry <= 2:
                    imagenRatonContento.dx = -2
                elif -2 <= self.position_horizontal_Jerry < -0.5:
                    imagenRatonContento.dx = 2
                else:
                    imagenRatonContento.dx = 0
                 
                if -0.2 < self.position_vertical_Jerry <= 2:
                    imagenRatonContento.dy = -2
                elif -2 <= self.position_vertical_Jerry < -0.2:
                    imagenRatonContento.dy = 2
                else:
                    imagenRatonContento.dy = 0
                
                
                grupoimagenRatonContento.update()
                grupoimagenGatoContento.update()
            
                # ... pero ¡recuerda!: Si en la posición nueva tenemos colisión con la pared
                # el movimiento no es posible, así que hay que deshacer el movimiento...
                # Si utilizamos cualquier función de colisión normal de pygame, la colisión
                # se produce si coincide la imagen del jugador con la de la pieza de la pared.
                # Pero esto no nos vale; de los dibujos al borde de las imagenes que los
                # contienen hay un margen y entonces no nos podríamos acercar del todo a las
                # paredes como en la realidad. Esto no queda bien, claro.
                # Podemos solucionarlo de dos formas; haciendo que las imágenes llenen por
                # completo su tamaño, de forma que no queden márgenes (algo que no siempre
                # es posible) o usando la función spritecollide junto con la función
                # collide_mask. Si lees la documentación de Pygame verás que lo que se consigue
                # con esto es que sólo se usen las partes reales del dibujo del sprite para
                # detectar la colisión. ¡Justo lo que queremos!
                # En el ejemplo que estamos escribiendo nos basta con esto. Pero debes saber
                # que estas funciones son incluso más potentes; permiten usar máscaras
                # específicas para indicar qué partes del sprite cuentan para colisionar y qué
                # partes no. ¿Te imaginas un sprite de un pirata con un cuchillo en el que
                # éste reaccione sólo cuando se choque a la altura del cuchillo?     
                
                if pygame.sprite.spritecollide(imagenRatonContento, nivel.grupo, 0, pygame.sprite.collide_mask):		
                    imagenRatonContento.deshacer()
                    
                if pygame.sprite.spritecollide(imagenGatoContento, nivel.grupo, 0, pygame.sprite.collide_mask):
                    imagenGatoContento.deshacer()    
                    # Lo dicho. Como hay colisión, hay que deshacer el movimiento.
                
                
                for pum in pygame.sprite.groupcollide(grupoimagenRatonContento, nivel.quesos, 0, 1):
                    pass   
            
                for pum in pygame.sprite.groupcollide(grupoimagenRatonContento, grupoimagenGatoContento, 1, 0):
                    pass     

                # Ya todo está en orden. Simplemente hay que dibujar cada elemento en sus
                # nuevas posiciones. En primer lugar el nivel del juego (en este programa
                # no cambia, pero en general podríamos tener elementos móviles):    
                
                nivel.actualizar(visor)
                
                # Y luego el sprite del jugador.    
                
                grupoimagenRatonContento.draw(visor)
                grupoimagenGatoContento.draw(visor)
                
                # ¡Se acabó! Lo volcamos en pantalla y ya está el fotograma completado.    
                
                pygame.display.update()
            
            
  
if __name__ == "__main__":
    game = juego()
    while not rospy.is_shutdown():
        game.loop()