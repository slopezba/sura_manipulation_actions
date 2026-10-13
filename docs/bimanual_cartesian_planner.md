# Planificación cartesiana bimanual

`PlanBimanualCartesianTrajectory` calcula una trayectoria articular conjunta para alcanzar la separación y orientación relativas entre los brazos A y B. Ambos participan sin prioridad por brazo; `planning.weights` permite penalizar individualmente el movimiento de cada articulación. La salida ejecutable es `trajectory_msgs/JointTrajectory`; los `nav_msgs/Path` sirven para representar el resultado. No activa tareas ni cambia controladores.

## Lanzamiento y lifecycle

Primero debe estar disponible el modelo del robot, su TF, sus estados articulares y la escena con los obstáculos actuales. La lectura requiere estados recientes de todas las articulaciones independientes de una variable del modelo, también las pasivas, incluidas las que no se mueven en este plan, porque también afectan a las colisiones. El estado de la base se mantiene fijo durante el cálculo.

```bash
ros2 launch sura_manipulation_actions manipulation_actions.launch.py \
  execution_backend:=controller use_rviz:=true
ros2 lifecycle set /plan_bimanual_cartesian_trajectory_lifecycle_action_server configure
ros2 lifecycle set /plan_bimanual_cartesian_trajectory_lifecycle_action_server activate
```

El launch no realiza estas transiciones ni activa ningún backend. `configure` crea la acción y los servicios, como en el servidor joint; `activate` habilita los intentos de ejecución; estando inactivo se puede preparar un plan, pero no despacharlo. `deactivate`, `cleanup` y `shutdown` cancelan el trabajo actual y descartan el plan pendiente. Para recargar configuración, usa `cleanup` y vuelve a configurar.

El launch común reúne las acciones articulares y cartesianas, incluyendo la
acción cartesiana de un brazo. El launch `manipulation_cartesian_actions.launch.py`
se conserva como entrada independiente para la acción bimanual; utiliza uno u
otro para evitar servidores duplicados.

La acción por defecto es `/cirtesub/manipulation/plan_bimanual_cartesian_trajectory`, de tipo `sura_manipulation_actions/action/PlanBimanualCartesianTrajectory`.

## Goal y restricciones

| Campo | Significado |
| --- | --- |
| `arm_a`, `arm_b` | Identificadores exactos de `arm_names`; se pueden intercambiar. |
| `relative_target` | Separación y rotación relativas expresadas en los ejes de `relative_reference_frame` (base); `header.frame_id` debe coincidir con ese frame y el quaternion debe ser unitario. |
| `relative_position_tolerance` | Tolerancia independiente en X/Y/Z del marco de referencia (base), en metros. |
| `relative_orientation_tolerance` | Tolerancia independiente del vector logaritmo de SO(3), en radianes, respecto a la orientación relativa deseada. |
| `cartesian_step`, `planning_time` | Resolución lineal y presupuesto temporal. Valores no positivos usan los defaults del YAML. |
| `velocity_scaling`, `acceleration_scaling` | Factores de límites articulares en `(0, 1]`; cero usa el valor correspondiente del YAML. Valores negativos, superiores a 1 o no finitos se rechazan. |
| `execute` | `false`: publicar y esperar el servicio de ejecución; `true`: intentar ejecutar después de planificar. |

Las tolerancias relativas cero son restricciones estrictas, con las precisiones numéricas configuradas (`1e-6 m` y `1e-6 rad` por defecto). El primer punto articular conserva exactamente la configuración medida; no se corrige una postura inicial fuera de límites o en colisión. Ningún brazo queda fijado por el orden A/B.

La velocidad se selecciona antes de planificar mediante `velocity_scaling` y
`acceleration_scaling`. Por ejemplo, `0.05` limita al 5 % de los máximos cargados
de cada articulación. Son factores independientes; cero conserva el default
correspondiente, actualmente `0.1` en el YAML Alpha. Se aplican a la temporización
y a la validación final, y quedan guardados con el plan pendiente para comprobar
la ejecución con los mismos límites. Los dos backends reciben los tiempos y
derivadas resultantes. Para cambiar la velocidad de un plan pendiente, envía
un goal nuevo. `planning_time` limita el cálculo, no la duración del movimiento.

Sean `p_A`, `R_A` y `p_B`, `R_B` las poses de las pinzas expresadas en la base.
El objetivo contiene `Δp = p_B - p_A` y `ΔR = R_B · R_Aᵀ`: son la separación
y la diferencia de orientación en los ejes de la base, no la posición absoluta
del brazo B ni `T_A⁻¹ · T_B`. Aumentar X en `0.01` pide un centímetro más de
separación sobre X de la base aunque gire el brazo A. La orientación identidad
pide pinzas con igual orientación. El error angular es `log(ΔR_obj · ΔR_actualᵀ)`
y sus tolerancias por componente se expresan en los ejes de la base.
Al intercambiar A y B, la separación cambia de signo y la rotación relativa
se invierte; el frame de referencia permanece igual.

`relative_reference_frame` debe ser un link del modelo que no se mueva con
ninguna de las articulaciones comandadas. Se fija a su pose medida al comenzar
el cálculo. Vacío selecciona el frame del modelo. El ejemplo Alpha configura
`cirtesub/base_link`.

La referencia relativa interpola linealmente la posición y usa SLERP para la orientación, también cuando solo cambia la orientación. Los pasos por defecto son `0.005 m` y `0.05 rad`. Las tolerancias se aplican durante todo el recorrido relativo. La referencia define un camino geométrico; no exige avance uniforme en tiempo. Al validar cada segmento se proyecta la pose relativa sobre ese tramo de la recta (o del arco SLERP en orientación pura), se acota el progreso a sus endpoints y se comprueban todas las componentes respecto a esa referencia.

## Pesos articulares

El servidor carga `planning.weights` al configurar su lifecycle. Cada valor añade
el coste `w_i · (q_i - q_inicial_i)²`: cuanto mayor sea, más se penaliza mover esa
articulación respecto a su posición medida al iniciar el plan. Un cero elimina
solo esta penalización adicional; no bloquea la articulación. Los costes generales
de continuidad, postura nominal y suavizado siguen aplicándose.

```yaml
# Orden: alpha_left axis_b,c,d,e; alpha_right axis_b,c,d,e
planning.weights: [0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0, 1.57]
```

Este ejemplo penaliza especialmente `axis_e` de ambos brazos. Los pesos se
asocian al orden de variables activas del grupo MoveIt conjunto, no al orden A/B
del goal ni a `planning.limit_joint_names`. En Alpha son las ocho articulaciones
indicadas arriba. Al configurar se registra cada índice, nombre y peso para
comprobar la correspondencia con el modelo cargado.

Un vector omitido o vacío equivale a pesos cero. Un vector no vacío debe tener
un valor por articulación activa, todos finitos y no negativos; una longitud
incorrecta, NaN, infinito o un valor negativo impiden configurar el servidor.
El planner también valida esos requisitos cuando se utiliza directamente.
Los pesos se conservan con el plan pendiente. Para modificarlos, edita el YAML,
reinicia el servidor para cargarlo y realiza `configure` otra vez. También puedes
hacer `cleanup`, actualizar el parámetro `planning.weights` y hacer `configure`;
`cleanup` descarta cualquier plan pendiente. Los goals nuevos usarán los pesos
cargados; intercambiar A/B conserva el peso de cada articulación física.

La interfaz usa `arm_a`/`arm_b` y devuelve `arm_a_path`/`arm_b_path`; los clientes
anteriores deben actualizarse y regenerar los mensajes de la acción.

## Ejemplo Alpha y plan pendiente

Los efectores del ejemplo son `cirtesub/alpha_left/standard_jaws_tool` y
`cirtesub/alpha_right/standard_jaws_tool`, tanto en el planificador como en el
cliente de ejemplo. El `frame_id` del objetivo es `cirtesub/base_link` para
ambos órdenes A/B.

El cliente de ejemplo obtiene ambas poses por TF en la base, con el mismo timestamp, y calcula su separación y rotación relativas. Sin desplazamiento comprueba el flujo con la relación actual; puede fallar si el robot ya está fuera de límites o en colisión. Un desplazamiento opcional se expresa sobre X de la base y su viabilidad depende de la postura. `--reference-frame` permite otro marco que coincida con la configuración del servidor.

```bash
ros2 run sura_manipulation_actions plan_bimanual_cartesian_example.py
# Proponer 1 cm sobre X de la base:
ros2 run sura_manipulation_actions plan_bimanual_cartesian_example.py --relative-dx 0.01
# Planificar con límites de velocidad y aceleración al 5 %:
ros2 run sura_manipulation_actions plan_bimanual_cartesian_example.py \
  --relative-dx 0.01 --velocity-scaling 0.05 --acceleration-scaling 0.05
# Intercambiar A y B:
ros2 run sura_manipulation_actions plan_bimanual_cartesian_example.py \
  --arm-a alpha_right --arm-b alpha_left
```

El feedback pasa a `waiting_for_execute` después de publicar el plan. El goal sigue abierto hasta ejecutar, cancelar o reemplazar el plan; la trayectoria ya puede consultarse en su tópico y en RViz. Desde otra terminal:

```bash
ros2 service call \
  /plan_bimanual_cartesian_trajectory_lifecycle_action_server/execute_pending_trajectory \
  std_srvs/srv/Trigger '{}'
ros2 service call \
  /plan_bimanual_cartesian_trajectory_lifecycle_action_server/cancel_pending_trajectory \
  std_srvs/srv/Trigger '{}'
```

La respuesta positiva del servicio de ejecución confirma que se solicitó el intento; el feedback y el resultado de la acción informan de la ejecución real. Si el backend está desactivado o no disponible, el plan permanece pendiente. Activar el backend no ejecuta nada: hay que llamar otra vez al servicio. `--execute` hace el intento inicial automáticamente tras planificar, con la misma política de activación manual.

Antes de despachar se comprueban nuevamente el estado inicial, la base y la escena. Los defaults admiten `0.001 rad` en joints revolute, `0.0001 m` en prismatic y desplazamientos de base de `0.005 m / 0.01 rad`. También se revalida la curva completa con las posiciones actuales de las articulaciones no comandadas y los obstáculos actuales. Un plan que ya no cumple estas condiciones se rechaza y exige nueva planificación. Cancelar una ejecución, un rechazo parcial o un fallo después del despacho cancela todos sus goals, incluidas las aceptaciones tardías.

## Activación manual de los backends

Para `controller`, el operador debe cargar y activar ambos `JointTrajectoryController` y desactivar los controladores incompatibles que reclamen esas interfaces. Consulta primero el estado real:

```bash
ros2 control list_controllers -c /cirtesub/alpha/controller/controller_manager
```

Por ejemplo, si los dos controladores forward de velocidad son los que están activos:

```bash
ros2 control switch_controllers -c /cirtesub/alpha/controller/controller_manager \
  --deactivate alpha_left_forward_velocity_controller alpha_right_forward_velocity_controller \
  --activate alpha_left_joint_trajectory_controller alpha_right_joint_trajectory_controller
```

Adapta la lista de desactivación a los controladores activos. El YAML define los nombres, servicios `list_controllers` y actions de cada brazo. El servidor solo consulta su estado. Divide la trayectoria por nombres articulares, conservando posiciones, velocidades, aceleraciones y tiempos. Envía ambos goals antes de esperar respuestas, con una fecha de inicio común tres segundos en el futuro y dos segundos de plazo de aceptación. Solo informa éxito cuando ambos resultados lo confirman. El backend debe respetar los stamps y la interpolación quíntica especificada por posiciones, velocidades y aceleraciones.

Para `task`:

```bash
ros2 launch sura_manipulation_actions manipulation_cartesian_actions.launch.py \
  execution_backend:=task use_rviz:=true
```

El bringup carga `task_priority_controller` sin activarlo. Si `list_controllers` muestra `unconfigured`, configúralo primero:

```bash
ros2 control set_controller_state task_priority_controller inactive \
  -c /cirtesub/alpha/controller/controller_manager
```

Prepara la selección de tareas mientras el controlador está inactivo.

Selecciona `bimanual_joint_trajectory`. Esa tarea está añadida a `ros2_control_params_dual_alpha.yaml`, inicialmente deshabilitada, con los ocho joints del grupo `bimanual` y `interpolation_method: quintic`. Desactiva las tareas incompatibles antes de habilitarla. El ejemplo Alpha también tiene `left_pose` y `right_pose` activos con prioridad mayor: pueden bloquear el seguimiento aunque las otras tareas de trayectoria estén desactivadas. Por ejemplo:

```bash
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: left_joint_trajectory, enabled: false}'
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: right_joint_trajectory, enabled: false}'
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: left_pose, enabled: false}'
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: right_pose, enabled: false}'
ros2 service call /set_task_enabled task_priority_kinematic_control/srv/SetTaskEnabled \
  '{task_id: bimanual_joint_trajectory, enabled: true}'
```

Cuando esté configurado, activa manualmente `task_priority_controller` desactivando los controladores incompatibles. Por ejemplo, para pasar desde los dos JTC activos:

```bash
ros2 control switch_controllers -c /cirtesub/alpha/controller/controller_manager \
  --deactivate alpha_left_joint_trajectory_controller alpha_right_joint_trajectory_controller \
  --activate task_priority_controller
```

También puedes hacerlo desde el panel de tareas. El nombre del servicio anterior corresponde al plugin de controlador del workspace; si usas el runtime independiente, consulta su namespace. La ruta del backend conjunto es `/cirtesub/controller/task_priority/tasks/bimanual_joint_trajectory/follow_joint_trajectory`, remapeada por el bringup a partir de `task_ids`.

Se envía una única trayectoria completa a esa tarea, con stamp cero: los dos brazos comparten el reloj y el avance de la misma tarea. `JointTrajectoryTask` usa el reloj monotónico existente; con simulación, su avance sigue tiempo real. No reproduce fechas futuras de ROS como los JTC. La tarea rechaza goals estando deshabilitada, y el plugin los rechaza si el controlador no está activo. Cada tarea admite un único goal simultáneo; cancelar, deshabilitar o desactivar el controlador borra la trayectoria almacenada. Reactivar no reanuda nada. `hold_last_point=true` mantiene la postura después del éxito hasta cancelar, deshabilitar, desactivar o reemplazar la trayectoria. El éxito libera el goal para admitir otro. Las tareas existentes conservan interpolación `linear` por defecto.

## Configurar otros brazos

Usa una copia de `config/bimanual_cartesian_planner.yaml`. Cada brazo requiere un grupo MoveIt de cadena serial, un tip afectado por ese grupo y su endpoint `FollowJointTrajectory`. El grupo de cada pareja debe contener exactamente la unión de las articulaciones activas de sus dos brazos, sin joints compartidos. Se admiten distintos números de articulaciones actuables revolute y prismatic de una variable; los joints pasivos dentro del grupo comandado se rechazan; las cadenas con articulaciones mimic que afecten al tip se rechazan. La configuración se valida al cargar el modelo y al planificar.

Ejemplo de estructura, dentro de `/**: ros__parameters:`:

```yaml
arm_names: [arm_a, arm_b]
relative_reference_frame: robot_base
pair_names: [pair_ab]
arms.arm_a.group: group_a
arms.arm_a.tip_frame: tool_a
arms.arm_a.controller_action: /a/follow_joint_trajectory
arms.arm_a.controller_name: a_controller
arms.arm_a.manager_service: /controller_manager/list_controllers
arms.arm_b.group: group_b
arms.arm_b.tip_frame: tool_b
arms.arm_b.controller_action: /b/follow_joint_trajectory
arms.arm_b.controller_name: b_controller
arms.arm_b.manager_service: /controller_manager/list_controllers
pairs.pair_ab.arms: [arm_a, arm_b]
pairs.pair_ab.group: both_groups
pairs.pair_ab.task_action: /tasks/combined/follow_joint_trajectory
pairs.pair_ab.task_controller_name: task_priority_controller
pairs.pair_ab.task_manager_service: /controller_manager/list_controllers
planning.limit_joint_names: []
planning.lower_limits: []
planning.upper_limits: []
```

Configura también `joint_states_topic`, `relative_reference_frame`, los efectores y los límites de velocidad y aceleración del nuevo modelo. La referencia debe ser un link independiente de los brazos. Para el backend task, configura una única `JointTrajectoryTask` con todos esos joints e interpolación quíntica en su propio paquete de robot. No hay tamaños articulares fijos en el algoritmo.

El launch admite `planner_config_file`, `robot_description_package/file`, `robot_description_semantic_package/file`, `xacro_arguments`, `srdf_xacro_arguments`, `moveit_config_package`, `joint_limits_file`, `kinematics_file`, `joint_states_topic`, `planning_scene_input_topic`, `relative_reference_frame`, `fixed_frame` y `rviz_config_file`. Los paths pueden ser absolutos o relativos al paquete correspondiente. Cambia conjuntamente URDF, SRDF, límites, configuración y RViz:

```bash
ros2 launch sura_manipulation_actions manipulation_cartesian_actions.launch.py \
  robot_namespace:=my_robot planner_config_file:=/ruta/planner.yaml \
  robot_description_package:=my_description robot_description_file:=urdf/robot.urdf.xacro \
  robot_description_semantic_package:=my_description \
  robot_description_semantic_file:=config/robot.srdf \
  xacro_arguments:='' srdf_xacro_arguments:='' \
  moveit_config_package:=my_description joint_limits_file:=config/limits.yaml \
  kinematics_file:=config/kinematics.yaml fixed_frame:=world \
  joint_states_topic:=/my_robot/joint_states rviz_config_file:=/ruta/my_robot.rviz
```

El launch expande `${robot_namespace}` en los valores del YAML del planificador. El YAML MoveIt existente de Alpha y el RViz de ejemplo contienen nombres `cirtesub/...`: al cambiar namespace, adapta también esos archivos o proporciona versiones propias. Los límites adicionales del ejemplo proceden de `tasks.joint_limits` de la configuración de seguridad Alpha, se mapean por nombre y se intersectan con el modelo cargado; solo afectan a este planificador. Una postura nominal opcional se define mediante `planning.nominal_joint_names` y `planning.nominal_positions`; por defecto se usa la inicial.

## RViz y salidas

`rviz/bimanual_cartesian_planning.rviz` incluye la escena MoveIt y la animación del robot completo. Con el frame fijo `world_ned`, utiliza:

| Tópico por defecto | Tipo |
| --- | --- |
| `/cirtesub/moveit/monitored_planning_scene` | `moveit_msgs/PlanningScene` |
| `/cirtesub/moveit/display_planned_path` | `moveit_msgs/DisplayTrajectory` con estado inicial y trayectoria completa |
| `/cirtesub/bimanual_planner/joint_trajectory` | `trajectory_msgs/JointTrajectory`, salida principal |
| `/cirtesub/bimanual_planner/arm_a_path`, `arm_b_path` | `nav_msgs/Path` alcanzados por FK, expresados en la base/referencia |
| `/cirtesub/bimanual_planner/relative_path`, `desired_relative_path` | `nav_msgs/Path` relativo alcanzado y deseado, dibujados con offset del brazo A inicial en la base |
| `/cirtesub/manipulation/markers` | Un único `visualization_msgs/MarkerArray` por plan con ejes del objetivo y de las poses finales y líneas de los brazos A/B, relativo planificado y deseado |

Las acciones individual y bimanual comparten ese tópico y el mismo
estilo. El path planificado es verde y el deseado violeta; en bimanual son
los paths relativos, con el offset explicado abajo. También aparecen el
brazo A cian y el brazo B naranja. La línea deseada es más ancha y translúcida
para distinguirla cuando coincide con la planificada. Las poses muestran
X rojo, Y verde y Z azul: objetivo con ejes de 6 cm, poses finales con ejes
de 4 cm. No se publica la antigua flecha amarilla del objetivo individual.

RViz usa un único display `MarkerArray` para ese tópico común. El publicador
común envía todas las capas habilitadas del último plan juntas en un solo mensaje
retenido mediante `transient_local`. Los namespaces
`bimanual_cartesian/arm_a_path`, `arm_b_path`, `planned_path`, `desired_path`,
`target`, `arm_a_final` y `arm_b_final` permiten ocultar
capas. La acción individual usa `cartesian/...`. Un plan nuevo sustituye
la visualización anterior, aunque sea de otra acción. El nodo común
`manipulation_marker_visualization` recibe las instantáneas completas en
`/cirtesub/manipulation/marker_plans` y es el único publicador del tópico RViz.
Publica `DELETEALL` seguido del nuevo plan en el mismo mensaje, usando un
namespace vacío para el borrado global e ID reservado, con parejas namespace/ID
únicas. La salida
`transient_local`, profundidad 1, permite abrir RViz después y recibir solo
el plan más reciente.
Los mensajes `nav_msgs/Path` conservan sus tópicos individuales para no mezclar
los datos de distintos planes en el refresco.

La trayectoria, paths, markers y `DisplayTrajectory` usan QoS fiable `transient_local`.
Todos los paths y markers se expresan en el frame existente `relative_reference_frame`
(`cirtesub/base_link` en Alpha). La acción no crea frames ni publica TF adicionales.
Los paths alcanzados salen de la FK de los waypoints finales temporizados.

Para mantener el dibujo junto a los brazos, los paths relativos llevan la posición
inicial del brazo A como offset numérico en ese frame:
`p_dibujo = p_A_inicial + Δp`. Sus orientaciones siguen representando `ΔR`,
no la orientación absoluta de la pinza. Para recuperar la separación a partir
del Path, resta la posición inicial del brazo A expresada en la base. El goal
`relative_target` conserva sus valores `Δp` y `ΔR`, sin ese offset de dibujo.
El marker del objetivo muestra la pose del brazo B que satisfaría esa relación
manteniendo la pose inicial del brazo A; si el brazo A se mueve, puede diferir de la
pose final real.

Los paths llevan timestamp cero y se refrescan a 10 Hz para que RViz use el TF
más reciente incluso con `world_ned` como frame fijo. Los markers tienen
`frame_locked=true`. Si la base se desplaza o gira, la representación acompaña
a la base; mover la pinza A no desplaza el origen del dibujo. Se requiere
el TF existente entre la base y el frame fijo de RViz. Cada nuevo plan limpia
todos los markers anteriores antes de añadir los suyos, dentro del mismo mensaje.
Las líneas comunes también son frame-locked y no necesitan el
refresco periódico para seguir a la base. El refresco de los `nav_msgs/Path`
solo publica paths: no vuelve a planificar
ni envía trayectorias articulares para ejecutar.

Los launches común y cartesiano arrancan el publicador de visualización
automáticamente. Mantén uno por robot; `launch_marker_visualization:=false`
permite reutilizar uno existente. La entrada es volátil: el nodo debe estar
activo al publicar el plan; si se reinicia solo ese nodo, publica un nuevo plan.
Reabrir RViz no requiere nueva planificación mientras el nodo permanece activo.

La planificación sigue suponiendo la base fija. Antes de ejecutar se comprueba
el movimiento de la base y la escena; superar las tolerancias configuradas
invalida el plan y requiere uno nuevo. Que el dibujo acompañe a la base no
revalida la trayectoria respecto a obstáculos del mundo. La escena recibe
actualizaciones en `/cirtesub/moveit/planning_scene`.

## Método y resolución de validación

Los módulos separan contexto de modelo/escena, planificación, visualización y transporte; la temporización tiene su módulo propio y el planificador valida los segmentos geométricos. La optimización secuencial usa Eigen y OSQP (`osqp_vendor`), FK y jacobianos de MoveIt, transformados desde el marco de cada cadena al marco común del modelo. Minimiza error relativo, cambios entre waypoints, movimiento articular ponderado respecto al inicio y distancia a la postura nominal. Los pesos no sustituyen la aceptación por límites articulares, tolerancias relativas y colisiones.

Se usa la solución anterior como semilla, hasta 100 iteraciones, ocho semillas locales deterministas y seis niveles de subdivisión; el máximo es 2.000 waypoints y se respeta el presupuesto del goal. El suavizado mantiene bloqueado el primer punto y solo acepta cambios que conservan restricciones. Se temporiza con `IterativeParabolicTimeParameterization` y los factores efectivos del goal (defaults Alpha `0.1 / 0.1`); después se valida la curva quíntica que se ejecutará. Puede subdividir, volver a temporizar y dilatar uniformemente el tiempo para cumplir los límites dinámicos. Los puntos incluyen q/v/a y tiempos estrictamente crecientes desde cero.

Antes de temporizar, cada segmento articular se muestrea con al menos cuatro subdivisiones, aumentando según el desplazamiento máximo por joint (`0.01 rad` o `0.001 m`). En la curva quíntica se comprueban analíticamente extremos de posición, velocidad y aceleración. FK, restricciones cartesianas y colisiones se muestrean con al menos cuatro subdivisiones, con intervalos máximos de `0.01 s` y desplazamientos por joint acotados por la velocidad máxima analítica y las resoluciones anteriores. Los límites de muestras por segmento son 4.096 y 20.000 respectivamente; superarlos produce fallo. Estos parámetros son configurables.

La validación geométrica tiene resolución numérica finita, no constituye una prueba de colisión continua. Un fallo de búsqueda significa que no se encontró una solución válida dentro del presupuesto, no que la pose sea globalmente inalcanzable. Los obstáculos que cambien después del despacho requieren la supervisión de ejecución del sistema existente.

## Pruebas y compilación por el usuario

Se han añadido suites C++ de planificador, transporte y servidor, y regresiones del backend task. Cubren brazos con distinto número de joints, A/B intercambiados, reparto del movimiento, pesos iguales, cero e inválidos, límites articulares, tolerancias, continuidad, colisiones entre puntos, temporización, FK y visualización, flujo pendiente, backend desactivado, inicio común, rechazo parcial, cancelación y ausencia de reanudación.

Las regresiones nuevas de referencia en la base incluyen referencia girada,
orientaciones no conmutativas, intercambio A/B, jacobiano relativo al mover el
brazo A, rechazo de referencias móviles y validación del frame en el servidor.
La implementación de este cambio se comprobó sin compilar; esas pruebas C++
quedan pendientes de la compilación y ejecución por el usuario.

Desde la raíz del workspace, el usuario puede compilar los paquetes y ejecutar las pruebas C++:

```bash
cd /home/salva/sura_ws_meta
colcon build --packages-select sura_manipulation_actions task_priority_kinematic_control_rqt
source install/setup.bash
colcon test --packages-select sura_manipulation_actions task_priority_kinematic_control_rqt
colcon test-result --verbose
```
