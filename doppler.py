import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
from matplotlib.patches import Ellipse
import threading
import sounddevice as sd

# ---------------------------
# Parámetros iniciales
# ---------------------------

# Velocidad del sonido en el aire (m/s)
v_sonido = 343

# Frecuencia emitida por la fuente (Hz)
f_fuente = 440  # Nota La (A4)

# Posiciones iniciales (en metros)
pos_fuente_ini = np.array([-50.0, 0.0])
pos_observador_ini = np.array([50.0, 0.0])

# Velocidades iniciales (en m/s)
vel_fuente_ini = np.array([20.0, 0.0])       # Fuente moviéndose hacia la derecha
vel_observador_ini = np.array([0.0, 0.0])    # Observador en reposo

# ---------------------------
# Configuración de la figura y controles
# ---------------------------

fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(left=0.1, bottom=0.35)  # Ajustar espacio para controles

ax.set_xlim(-100, 100)
ax.set_ylim(-100, 100)
ax.set_xlabel('Posición X (m)')
ax.set_ylabel('Posición Y (m)')
ax.set_title('Simulación del Efecto Doppler con Sonido')
ax.grid()

# Elementos gráficos
punto_fuente, = ax.plot([], [], 'bo', label='Fuente', markersize=10, picker=5, zorder=5)
punto_observador, = ax.plot([], [], 'ro', label='Observador', markersize=10, picker=5, zorder=5)
texto_frecuencia = ax.text(0.02, 0.80, '', transform=ax.transAxes, fontsize=10, va='top')
ax.legend()

# Mostrar la ecuación del efecto Doppler (estática)
texto_ecuacion = ax.text(0.02, 0.95, r"$f' = f \left( \dfrac{v + v_o}{v - v_s} \right)$", transform=ax.transAxes, fontsize=12, va='top')

# Lista para almacenar las ondas emitidas
ondas_emergentes = []

# Definir los ejes para los sliders
axcolor = 'lightgoldenrodyellow'

# Sliders para la Fuente
ax_vel_fuente_x = plt.axes([0.1, 0.24, 0.35, 0.02], facecolor=axcolor)
ax_vel_fuente_y = plt.axes([0.1, 0.20, 0.35, 0.02], facecolor=axcolor)

# Sliders para el Observador
ax_vel_obs_x = plt.axes([0.6, 0.24, 0.35, 0.02], facecolor=axcolor)
ax_vel_obs_y = plt.axes([0.6, 0.20, 0.35, 0.02], facecolor=axcolor)

# Crear los sliders
s_vel_fuente_x = Slider(ax_vel_fuente_x, 'Vel Fuente X', -100, 100, valinit=vel_fuente_ini[0])
s_vel_fuente_y = Slider(ax_vel_fuente_y, 'Vel Fuente Y', -100, 100, valinit=vel_fuente_ini[1])

s_vel_obs_x = Slider(ax_vel_obs_x, 'Vel Obs X', -100, 100, valinit=vel_observador_ini[0])
s_vel_obs_y = Slider(ax_vel_obs_y, 'Vel Obs Y', -100, 100, valinit=vel_observador_ini[1])

# ---------------------------
# Funciones de interacción
# ---------------------------

# Variables globales para arrastrar
arrastrando_fuente = False
arrastrando_observador = False

def on_pick(event):
    global arrastrando_fuente, arrastrando_observador
    if event.artist == punto_fuente:
        arrastrando_fuente = True
    elif event.artist == punto_observador:
        arrastrando_observador = True

def on_release(event):
    global arrastrando_fuente, arrastrando_observador
    arrastrando_fuente = False
    arrastrando_observador = False

def on_motion(event):
    global pos_fuente, pos_observador
    if arrastrando_fuente and event.inaxes == ax:
        pos_fuente = np.array([event.xdata, event.ydata])
    elif arrastrando_observador and event.inaxes == ax:
        pos_observador = np.array([event.xdata, event.ydata])

# Conectar los eventos
fig.canvas.mpl_connect('pick_event', on_pick)
fig.canvas.mpl_connect('button_release_event', on_release)
fig.canvas.mpl_connect('motion_notify_event', on_motion)

# ---------------------------
# Funciones de cálculo y actualización
# ---------------------------

def calcular_frecuencia_percibida(f, v, v_o, v_s):
    return f * ((v + v_o) / (v - v_s))

def init():
    punto_fuente.set_data([pos_fuente_ini[0]], [pos_fuente_ini[1]])
    punto_observador.set_data([pos_observador_ini[0]], [pos_observador_ini[1]])
    texto_frecuencia.set_text('')
    return punto_fuente, punto_observador, texto_frecuencia

# Variables globales para posiciones y velocidades
pos_fuente = pos_fuente_ini.copy()
pos_observador = pos_observador_ini.copy()
vel_fuente = vel_fuente_ini.copy()
vel_observador = vel_observador_ini.copy()

# Lista para almacenar las ondas con su tiempo de emisión y artista correspondiente
ondas_emergentes = []

# Variable para controlar la pausa
pausado = False

# Variable para almacenar la flecha de velocidad de la fuente
flecha_velocidad_fuente = None

# Variable global para la frecuencia percibida
frecuencia_percibida = f_fuente

def update(frame):
    global pos_fuente, pos_observador, vel_fuente, vel_observador, ondas_emergentes, pausado, flecha_velocidad_fuente, frecuencia_percibida

    dt = 0.1
    tiempo_actual = frame * dt

    # Actualizar velocidades con los valores de los sliders
    vel_fuente = np.array([s_vel_fuente_x.val, s_vel_fuente_y.val])
    vel_observador = np.array([s_vel_obs_x.val, s_vel_obs_y.val])

    if not pausado:
        # Actualizar posiciones
        pos_fuente += vel_fuente * dt
        pos_observador += vel_observador * dt

        # Emitir nueva onda desde la posición actual de la fuente
        nueva_onda = {
            'centro': pos_fuente.copy(),
            'tiempo_emision': tiempo_actual,
            'angulo': np.arctan2(vel_fuente[1], vel_fuente[0]) if np.linalg.norm(vel_fuente) != 0 else 0,
            'velocidad_fuente': vel_fuente.copy(),
            'artista': None  # Se creará más adelante
        }
        ondas_emergentes.append(nueva_onda)

    # Actualizar las ondas existentes
    for onda in ondas_emergentes[:]:
        tiempo_desde_emision = tiempo_actual - onda['tiempo_emision']
        distancia_onda = v_sonido * tiempo_desde_emision

        # Crear/actualizar la elipse que representa la onda
        velocidad_relativa = np.linalg.norm(onda['velocidad_fuente']) / v_sonido
        factor_distorsion = max(1 - velocidad_relativa, 0.1)

        width = distancia_onda * 2
        height = distancia_onda * 2 * factor_distorsion

        if onda['artista'] is None:
            # Crear el artista para la onda
            elipse = Ellipse(xy=onda['centro'], width=width, height=height, angle=np.degrees(onda['angulo']), edgecolor='blue', facecolor='none', alpha=0.5, zorder=1)
            onda['artista'] = elipse
            ax.add_patch(elipse)
        else:
            # Actualizar el artista existente
            elipse = onda['artista']
            elipse.width = width
            elipse.height = height
            elipse.angle = np.degrees(onda['angulo'])
            # El centro permanece igual

        # Remover ondas que ya no son visibles
        if distancia_onda > 500:
            elipse.remove()
            ondas_emergentes.remove(onda)

    # Distancia y dirección entre fuente y observador
    vector_dif = pos_observador - pos_fuente
    distancia = np.linalg.norm(vector_dif)
    if distancia == 0:
        distancia = 1e-10
    direccion = vector_dif / distancia

    # Proyecciones de velocidad en la dirección de propagación
    v_s = np.dot(vel_fuente, direccion)
    v_o = np.dot(vel_observador, direccion)

    # Calcular frecuencia percibida
    frecuencia_percibida = calcular_frecuencia_percibida(f_fuente, v_sonido, v_o, v_s)

    # Actualizar posiciones de los puntos
    punto_fuente.set_data([pos_fuente[0]], [pos_fuente[1]])
    punto_observador.set_data([pos_observador[0]], [pos_observador[1]])

    # Ajustar los límites de los ejes para mantener los puntos visibles
    min_x = min(pos_fuente[0], pos_observador[0]) - 50
    max_x = max(pos_fuente[0], pos_observador[0]) + 50
    min_y = min(pos_fuente[1], pos_observador[1]) - 50
    max_y = max(pos_fuente[1], pos_observador[1]) + 50
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)

    # Actualizar texto de frecuencia percibida con fórmula y valores actuales
    texto_frecuencia.set_text(
        fr"$f' = {f_fuente:.0f} \left( \dfrac{{{v_sonido:.0f} + {v_o:.2f}}}{{{v_sonido:.0f} - {v_s:.2f}}} \right) = {frecuencia_percibida:.2f}\ \text{{Hz}}$"
    )

    # Remover flecha de velocidad anterior
    if flecha_velocidad_fuente is not None:
        flecha_velocidad_fuente.remove()

    # Dibujar flecha de velocidad de la fuente
    flecha_velocidad_fuente = ax.arrow(pos_fuente[0], pos_fuente[1], vel_fuente[0]*0.5, vel_fuente[1]*0.5, head_width=5, head_length=10, fc='blue', ec='blue', zorder=2)

    return punto_fuente, punto_observador, texto_frecuencia

def actualizar(val):
    # Callback vacío para actualizar la animación al mover los sliders
    pass

# Conectar los sliders con la función 'actualizar'
s_vel_fuente_x.on_changed(actualizar)
s_vel_fuente_y.on_changed(actualizar)
s_vel_obs_x.on_changed(actualizar)
s_vel_obs_y.on_changed(actualizar)

# ---------------------------
# Funciones de sonido
# ---------------------------

def reproducir_sonido():
    global pausado, frecuencia_percibida

    fs = 44100  # Frecuencia de muestreo
    duration = 0.1  # Duración de cada fragmento de sonido (segundos)

    while True:
        if not pausado:
            # Generar tono con la frecuencia percibida actual
            t = np.linspace(0, duration, int(fs * duration), endpoint=False)
            tono = np.sin(2 * np.pi * frecuencia_percibida * t) * 0.5  # Amplitud reducida para evitar saturación

            # Reproducir el tono
            sd.play(tono, fs)
            sd.wait()
        else:
            sd.stop()
            # Esperar un momento para evitar uso excesivo de CPU
            sd.sleep(100)

# Iniciar el hilo de sonido
hilo_sonido = threading.Thread(target=reproducir_sonido, daemon=True)
hilo_sonido.start()

# ---------------------------
# Botones de control
# ---------------------------

# Botón de pausa/reproducción
ax_play = plt.axes([0.45, 0.025, 0.1, 0.04])
button_play = Button(ax_play, 'Pausa', color=axcolor, hovercolor='0.975')

def play_pause(event):
    global pausado
    pausado = not pausado
    if pausado:
        button_play.label.set_text('Reanudar')
    else:
        button_play.label.set_text('Pausa')

button_play.on_clicked(play_pause)

# Botón para reiniciar la animación
resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reiniciar', color=axcolor, hovercolor='0.975')

def reset(event):
    s_vel_fuente_x.reset()
    s_vel_fuente_y.reset()
    s_vel_obs_x.reset()
    s_vel_obs_y.reset()
    # Reiniciar posiciones
    global pos_fuente, pos_observador, ondas_emergentes, flecha_velocidad_fuente, frecuencia_percibida
    pos_fuente = pos_fuente_ini.copy()
    pos_observador = pos_observador_ini.copy()
    # Actualizar puntos
    punto_fuente.set_data([pos_fuente[0]], [pos_fuente[1]])
    punto_observador.set_data([pos_observador[0]], [pos_observador[1]])
    # Limpiar ondas
    for onda in ondas_emergentes:
        if onda['artista'] is not None:
            onda['artista'].remove()
    ondas_emergentes.clear()
    # Remover la flecha de velocidad
    if flecha_velocidad_fuente is not None:
        flecha_velocidad_fuente.remove()
        flecha_velocidad_fuente = None
    # Reiniciar frecuencia percibida
    frecuencia_percibida = f_fuente
    # Reiniciar animación
    anim.frame_seq = anim.new_frame_seq()
    # Resetear pausa
    global pausado
    pausado = False
    button_play.label.set_text('Pausa')

button.on_clicked(reset)

# ---------------------------
# Ejecutar la animación
# ---------------------------

anim = FuncAnimation(fig, update, frames=range(2000), init_func=init, blit=False, interval=50)
plt.show()
