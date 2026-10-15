# sura_manipulation_actions

Paquete ROS 2 para planificar y ejecutar trayectorias de manipuladores usando MoveIt 2 y `ros2_control`. Incluye acciones articulares, cartesianas de un brazo y cartesianas bimanuales configurables.

`manipulation_actions.launch.py` reúne los cuatro servidores principales en un
solo launch. Puedes añadir `use_rviz:=true use_rqt:=true`; lifecycle y backend
siguen con activación manual. La pestaña **Cartesian** permite elegir brazo,
frame, ejes disponibles de base/articulaciones y límites por goal. El backend
`task` coordina `BasePoseTrajectoryTask` con la trayectoria del brazo y mantiene
la pose final; el backend `controller` admite movimientos con base bloqueada.
El backend `pose`, seleccionable como **Pose** en RQT, publica poses de
`standard_jaws_tool` directamente en `left_pose` o `right_pose` y deja que la
jerarquía resuelva el movimiento, sin IK ni validación previa de colisiones.
Consulta [la guía cartesiana de un brazo](docs/single_arm_cartesian_planner.md)
para lanzamiento, configuración, servicios y selección de pose/velocidad.

La nueva acción `PlanBimanualCartesianTrajectory`, su configuración y el flujo de activación manual se describen en [la guía cartesiana bimanual](docs/bimanual_cartesian_planner.md). Los apartados siguientes describen las acciones articulares existentes.

El nodo principal es un `LifecycleNode` que expone una action propia, `PlanJointTrajectory`, para mandar una pose predefinida o una posicion objetivo de 4 articulaciones a un brazo. El servidor:

- Carga el modelo URDF/SRDF y la configuracion de MoveIt.
- Lee el estado actual desde `/cirtesub/alpha/joint_states`.
- Carga poses articulares predefinidas desde YAML durante `configure`.
- Comprueba limites articulares y colisiones antes de planificar.
- Planifica con OMPL para los grupos MoveIt `alpha_left` o `alpha_right`.
- Publica la trayectoria planificada para verla en RViz.
- Cambia automaticamente del controlador de velocidad al controlador de trayectoria del brazo seleccionado.
- Ejecuta la trayectoria inmediatamente o la deja pendiente hasta recibir una confirmacion por servicio.

## Compilar

Desde la raiz del workspace:

```bash
colcon build --packages-select sura_manipulation_actions
source install/setup.bash
```

## Arrancar el servidor de acciones

```bash
ros2 launch sura_manipulation_actions manipulation_actions.launch.py
```

Con RViz en la misma orden:

```bash
ros2 launch sura_manipulation_actions manipulation_actions.launch.py use_rviz:=true
```

Argumentos utiles del launch:

```bash
ros2 launch sura_manipulation_actions manipulation_actions.launch.py \
  robot_namespace:=cirtesub \
  moveit_namespace:=moveit \
  default_planning_time:=5.0 \
  default_goal_tolerance:=0.02 \
  predefined_poses_file:=/ruta/a/predefined_poses.yaml \
  use_sim_time:=false
```

O también puedes lanzarlo como:
```bash
ros2 launch sura_manipulation_actions manipulation_actions.launch.py execution_backend:=task
```

## Hybrid Planning

Para usar la arquitectura Hybrid Planning de MoveIt:

```bash
ros2 launch sura_manipulation_actions manipulation_hybrid_planning_actions.launch.py
```

La action expuesta es:

```text
/cirtesub/manipulation/hybrid_plan_joint_trajectory
```

Usa el mismo tipo `sura_manipulation_actions/action/PlanJointTrajectory`. En este modo Hybrid Planning planifica y ejecuta online mediante el planner global y local de MoveIt, por lo que el campo `execute` se acepta por compatibilidad pero no deja una trayectoria pendiente.

```bash
ros2 launch sura_manipulation_actions manipulation_hybrid_planning_actions.launch.py execution_backend:=controller
ros2 launch sura_manipulation_actions manipulation_hybrid_planning_actions.launch.py execution_backend:=task
```

`execution_backend:=controller` publica la solucion local hacia los `JointTrajectoryController`; `execution_backend:=task` la publica hacia las tareas `left_joint_trajectory` y `right_joint_trajectory` del task priority controller.

El servidor hybrid queda disponible al arrancar el launch; no necesita `ros2 lifecycle set configure/activate`.
Este modo no cambia controladores automaticamente; deja activo antes el JTC o task priority que quieras usar. Para evitar bloqueos durante pruebas, el wrapper cancela el goal si MoveIt Hybrid Planning no devuelve resultado antes de `hybrid_result_timeout`.

Por defecto usa la descripcion de `cirtesub_description`:

- URDF: `urdf/cirtesub_dual_alpha.urdf.xacro`
- SRDF: `moveit2/cirtesub_dual_alpha.srdf.xacro`
- Configuracion MoveIt: `moveit2/planning_pipelines.yaml`, `ompl_planning.yaml`, `joint_limits.yaml`, `kinematics.yaml`

## Lifecycle: configure y activate

El nodo se llama:

```text
/plan_joint_trajectory_lifecycle_action_server
```

Despues de lanzar el nodo, pasalo a `configure`:

```bash
ros2 lifecycle set /plan_joint_trajectory_lifecycle_action_server configure
```

Esto inicializa MoveIt, los clientes de controladores, la action y los servicios de ejecucion/cancelacion.

Luego puedes pasarlo a `activate`:

```bash
ros2 lifecycle set /plan_joint_trajectory_lifecycle_action_server activate
```

Comprobar estado:

```bash
ros2 lifecycle get /plan_joint_trajectory_lifecycle_action_server
```

Para parar una ejecucion activa o limpiar recursos:

```bash
ros2 lifecycle set /plan_joint_trajectory_lifecycle_action_server deactivate
ros2 lifecycle set /plan_joint_trajectory_lifecycle_action_server cleanup
```

## Action disponible

Nombre por defecto:

```text
/cirtesub/manipulation/plan_joint_trajectory
```

Tipo:

```text
sura_manipulation_actions/action/PlanJointTrajectory
```

Goal:

```text
string arm_name
string predefined_pose
float64[4] joint_positions
float64 planning_time
float64 goal_tolerance
bool execute
```

Result:

```text
bool success
string message
float64 final_error_norm
```

Feedback:

```text
string state
string message
float64 closest_distance
string closest_object
float64[4] joint_error
```

`arm_name` acepta:

- Brazo izquierdo: `left`, `alpha_left`, `cirtesub/alpha_left`
- Brazo derecho: `right`, `alpha_right`, `cirtesub/alpha_right`

`predefined_pose` es opcional. Si no esta vacio, tiene prioridad sobre `joint_positions` y se busca en el YAML de poses predefinidas para el brazo indicado. El nombre se normaliza quitando espacios y pasando a minusculas.

`joint_positions` son 4 posiciones articulares en radianes. El gripper (`axis_a`) queda fuera de la planificacion y se controla por separado. Orden por defecto:

```text
cirtesub/alpha_left/axis_b
cirtesub/alpha_left/axis_c
cirtesub/alpha_left/axis_d
cirtesub/alpha_left/axis_e
```

o para el derecho:

```text
cirtesub/alpha_right/axis_b
cirtesub/alpha_right/axis_c
cirtesub/alpha_right/axis_d
cirtesub/alpha_right/axis_e
```

## Poses predefinidas

El archivo por defecto se instala en:

```text
share/sura_manipulation_actions/config/predefined_poses.yaml
```

Formato:

```yaml
predefined_poses:
  alpha_left:
    home: [0.0, 0.0, 0.0, 0.0]
    extend: [0.3, -0.4, 0.2, 0.0]
    fold: [-0.5, 0.8, -0.3, 0.0]
    unfold: [0.2, -0.2, 0.1, 0.0]

  alpha_right:
    home: [0.0, 0.0, 0.0, 0.0]
    extend: [-0.3, 0.4, -0.2, 0.0]
    fold: [0.5, -0.8, 0.3, 0.0]
    unfold: [-0.2, 0.2, -0.1, 0.0]
```

Tambien puedes pasar otro YAML desde launch:

```bash
ros2 launch sura_manipulation_actions manipulation_actions.launch.py \
  predefined_poses_file:=/ruta/a/mis_poses.yaml
```

El YAML se carga durante `configure`. Si cambias el archivo, haz `cleanup` y vuelve a hacer `configure`.

## Enviar un comando

Planificar una pose predefinida, publicar en RViz y esperar confirmacion antes de ejecutar:

```bash
ros2 action send_goal --feedback \
  /cirtesub/manipulation/plan_joint_trajectory \
  sura_manipulation_actions/action/PlanJointTrajectory \
  "{arm_name: left, predefined_pose: home, joint_positions: [0, 0, 0, 0], planning_time: 5.0, goal_tolerance: 0.02, execute: false}"
```

Planificar con posiciones manuales:

```bash
ros2 action send_goal --feedback \
  /cirtesub/manipulation/plan_joint_trajectory \
  sura_manipulation_actions/action/PlanJointTrajectory \
  "{arm_name: left, predefined_pose: '', joint_positions: [0.2, -0.3, 0.1, 0.0], planning_time: 5.0, goal_tolerance: 0.02, execute: false}"
```

Cuando la action quede en `waiting_for_execute`, ejecuta la trayectoria pendiente:

```bash
ros2 service call \
  /plan_joint_trajectory_lifecycle_action_server/execute_pending_trajectory \
  std_srvs/srv/Trigger
```

Planificar y ejecutar directamente:

```bash
ros2 action send_goal --feedback \
  /cirtesub/manipulation/plan_joint_trajectory \
  sura_manipulation_actions/action/PlanJointTrajectory \
  "{arm_name: right, predefined_pose: extend, joint_positions: [0, 0, 0, 0], planning_time: 5.0, goal_tolerance: 0.02, execute: true}"
```

Cancelar una trayectoria pendiente o activa:

```bash
ros2 service call \
  /plan_joint_trajectory_lifecycle_action_server/cancel_pending_trajectory \
  std_srvs/srv/Trigger
```

Tambien puedes cancelar desde la action si estas usando `ros2 action send_goal` en modo interactivo o desde un cliente propio.

## Visualizacion en RViz

Opcion rapida:

```bash
ros2 launch sura_manipulation_actions manipulation_actions.launch.py use_rviz:=true
```

O lanzar solo RViz con la configuracion del paquete:

```bash
ros2 launch sura_manipulation_actions manipulation_rviz.launch.py
```

La configuracion `rviz/manipulation_planning.rviz` incluye:

- `Fixed Frame`: `world_ned`
- `RobotModel`: `/cirtesub/robot_description`
- `MotionPlanning`: trayectoria en `/cirtesub/moveit/display_planned_path`
- `MotionPlanning`: planning scene en `/cirtesub/moveit/monitored_planning_scene`
- Un único display `MarkerArray` en `/cirtesub/manipulation/markers`: cada acción
  cartesiana publica sus poses y líneas juntas en un solo mensaje.
  Planificado verde, deseado violeta; en bimanual incluye brazo A cian y brazo B
  naranja, sin prioridad entre brazos. Las acciones articulares usan también este
  tópico para el debug de colisiones.

Las acciones cartesianas muestran ejes X rojo, Y verde y Z azul en el objetivo
(6 cm) y las poses finales (4 cm). Se pueden ocultar las capas por namespace
en el display. Un plan nuevo, incluso de otra acción, sustituye toda la
visualización anterior. El nodo `manipulation_marker_visualization` recibe las
instantáneas completas en `/cirtesub/manipulation/marker_plans` y es el único
publicador de `/cirtesub/manipulation/markers`. Añade `DELETEALL` con namespace
vacío e ID reservado al comienzo del mismo mensaje: en RViz, un namespace
no vacío limitaría el borrado a ese namespace. No se repiten las parejas namespace/ID.
Su salida fiable `transient_local`, con profundidad 1, conserva solo el último
plan para RViz, también si se abre después de planificar.
Los paths usan el frame de la base y
siguen su TF existente. Los `nav_msgs/Path` individuales continúan disponibles
para consultar los datos de cada acción sin mezclar los refrescos.

Los launches `manipulation_actions.launch.py` y `manipulation_cartesian_actions.launch.py`
arrancan este nodo automáticamente. Mantén un solo publicador común por robot;
si ya está ejecutándose, usa `launch_marker_visualization:=false` en el otro
launch. Si lanzas los servidores por separado, inicia primero:

```bash
ros2 run sura_manipulation_actions manipulation_marker_visualization.py \
  --ros-args -r __ns:=/cirtesub
```

La entrada contiene eventos nuevos y es volátil: reiniciar únicamente el nodo
de visualización requiere publicar un nuevo plan. Abrir o reiniciar RViz no
requiere volver a planificar. No se publica ningún TF adicional.

Si no ves el robot o la trayectoria, comprueba:

```bash
ros2 topic echo /cirtesub/moveit/display_planned_path --once
ros2 topic echo /cirtesub/moveit/monitored_planning_scene --once
ros2 topic echo /cirtesub/manipulation/markers --once
```

## Controladores necesarios

Antes de ejecutar, deben existir los controladores de `ros2_control` y el `controller_manager` debe estar disponible en:

```text
/cirtesub/controller/controller_manager/list_controllers
/cirtesub/controller/controller_manager/switch_controller
```

Controladores esperados por defecto:

```text
alpha_left_forward_velocity_controller
alpha_left_gripper_velocity_controller
alpha_left_joint_trajectory_controller
alpha_right_forward_velocity_controller
alpha_right_gripper_velocity_controller
alpha_right_joint_trajectory_controller
```

Action servers de trayectoria esperados:

```text
/cirtesub/alpha/controller/alpha_left_joint_trajectory_controller/follow_joint_trajectory
/cirtesub/alpha/controller/alpha_right_joint_trajectory_controller/follow_joint_trajectory
```

El servidor cambia automaticamente el brazo seleccionado a modo trayectoria: activa el `joint_trajectory_controller` y desactiva el `forward_velocity_controller` si estaba activo.

## Topics principales

Entrada de estado articular:

```text
/cirtesub/alpha/joint_states
```

Salida de trayectoria planificada para RViz:

```text
/cirtesub/moveit/display_planned_path
```

Planning scene publicado:

```text
/cirtesub/moveit/monitored_planning_scene
```

Debug de colisiones:

```text
/cirtesub/manipulation/markers
```

## Parametros importantes

Se pueden sobreescribir desde launch o por parametros ROS 2:

```text
robot_namespace: cirtesub
moveit_namespace: moveit
action_name: /cirtesub/manipulation/plan_joint_trajectory
joint_states_topic: /cirtesub/alpha/joint_states
display_trajectory_topic: /cirtesub/moveit/display_planned_path
planning_scene_topic: /cirtesub/moveit/monitored_planning_scene
collision_debug_topic: /cirtesub/manipulation/marker_plans
predefined_poses_file: share/sura_manipulation_actions/config/predefined_poses.yaml
left_group_name: alpha_left
right_group_name: alpha_right
default_planning_time: 5.0
default_goal_tolerance: 0.02
controller_wait_timeout: 5.0
controller_switch_timeout: 2.0
execution_feedback_rate: 10.0
current_state_timeout: 2.0
planning_scene_publish_rate: 10.0
```

## Flujo recomendado de prueba

1. Lanza la simulacion o el robot con `robot_state_publisher`, `joint_states`, `controller_manager` y los controladores Alpha cargados.
2. Lanza este paquete:

   ```bash
   ros2 launch sura_manipulation_actions manipulation_actions.launch.py use_rviz:=true
   ```

3. Configura y activa el nodo:

   ```bash
   ros2 lifecycle set /plan_joint_trajectory_lifecycle_action_server configure
   ros2 lifecycle set /plan_joint_trajectory_lifecycle_action_server activate
   ```

4. Envia un goal con `predefined_pose: home` y `execute: false`.
5. Revisa la trayectoria en RViz.
6. Si la trayectoria es correcta, llama a `execute_pending_trajectory`.

## Errores frecuentes

- `Timed out waiting for current planning scene state`: no llegan joint states al topic configurado.
- `Trajectory controller ... is not loaded`: falta cargar el controlador de trayectoria.
- `Controller action server is unavailable`: el controlador existe pero su action `follow_joint_trajectory` no esta disponible.
- `Unknown predefined_pose ...`: la pose no existe para el brazo indicado en el YAML cargado.
- `Target joint_positions violate joint limits`: alguna posicion objetivo esta fuera de los limites del URDF/MoveIt.
- `Current state is in collision` o `Target state is in collision`: MoveIt detecta colision en el estado actual u objetivo.
- `No pending trajectory is waiting for execute trigger`: no hay ningun plan pendiente, o ya fue ejecutado/cancelado.
