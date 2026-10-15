# Planificación cartesiana de base y un brazo

## Modo de referencias de pose

Selecciona **Pose** en la pestaña Cartesian de RQT, o envía un goal con
`execution_backend: pose`. Este modo genera únicamente el recorrido del efector:
posición lineal, orientación SLERP por el arco corto y un perfil temporal quíntico
que empieza y termina en reposo. No calcula IK ni valida colisiones o
alcanzabilidad antes de ejecutar. `desired_path` contiene el recorrido geométrico;
`path`, `joint_trajectory`, `base_trajectory` y `base_path` quedan vacíos.

La referencia se publica a 100 Hz en los topics existentes de `left_pose` o
`right_pose`, expresada en `planning_frame` (`world_ned` por defecto). Las tareas
controlan `cirtesub/alpha_left/standard_jaws_tool` y
`cirtesub/alpha_right/standard_jaws_tool`, tanto en los ejemplos como en el perfil
de arranque `ros2_control_params_dual_alpha.yaml`. El Jacobiano corresponde al
frame de la herramienta. Las tareas, sus prioridades y los pesos del solver
determinan cómo participan la base y los brazos. `available_dofs`, los límites de
base y las tolerancias articulares/base del goal no se utilizan en este modo;
RQT deshabilita esos controles.

Los parámetros `pose.max_linear_velocity`, `pose.max_angular_velocity`,
`pose.max_linear_acceleration` y `pose.max_angular_acceleration` tienen defaults
de 1.0 m/s, 0.5 rad/s, 0.4 m/s² y 0.2 rad/s². `velocity_scaling` y
`acceleration_scaling` escalan esos máximos; cero conserva los factores del YAML.
Limitan la referencia cartesiana; las velocidades reales dependen del solver y
del error de seguimiento. `cartesian_step` y `planning.angular_step` controlan
la resolución del path visualizado. Se conserva `planning.max_waypoints`.

Cada brazo configura `arms.<arm>.pose_task` y
`arms.<arm>.pose_controller_node`. Por defecto los topics se derivan del namespace
de ese nodo y de la tarea: `task_priority/tasks/<task>/target` y `/feedforward`.
El perfil dual Alpha conserva el nodo en
`/cirtesub/alpha/controller/task_priority_controller`, pero el bringup remapea
los topics a `/cirtesub/controller/task_priority/tasks/<task>/target` y
`/feedforward`. El YAML del planificador configura explícitamente esas rutas
mediante `arms.<arm>.pose_target_topic` y `arms.<arm>.pose_feedforward_topic`.
Para otra distribución de nodos o remapeos, adapta ambos parámetros.
El backend comprueba los parámetros del controlador y exige que su mundo y su
frame controlado coincidan con los del planificador, y que los topics tengan
suscriptores. La tarea seleccionada debe estar habilitada por el operador;
el planificador no la habilita ni cambia la jerarquía.

El recorrido geométrico y los ejes del objetivo de Pose se muestran también en
`/cirtesub/manipulation/markers`, con el mismo publicador común que los otros
planificadores. El productor envía un conjunto completo de markers `ADD` a
`/cirtesub/manipulation/marker_plans`; `manipulation_marker_visualization` limpia
el plan anterior y publica el nuevo conjunto retenido en el topic común.
El nodo debe estar activo al planificar (`launch_marker_visualization:=true`).

Se mantiene el protocolo `execute: false` y ejecución/cancelación del plan
pendiente. El objetivo se transforma y congela al planificar; si la herramienta
se ha desplazado fuera de `tip_path` antes de ejecutar se exige un nuevo plan.
Durante la ejecución se guarda y desactiva temporalmente
`auto_update_target_pose` de la tarea seleccionada. Junto con cada pose se publica
su velocidad lineal y angular de feedforward, con el mismo timestamp y frame.
Se calcula analíticamente: `v = (p_final - p_inicial) * ds/dt` y
`omega = rotacion_vectorial * ds/dt`, donde la rotación sigue el mismo arco corto
de SLERP y está expresada en los ejes del frame de planificación. Para
`u = t/T`, el perfil quíntico usa `ds/dt = 30*u²*(1-u)²/T`.
El feedforward es cero en los extremos, durante la convergencia final y cuando
el reloj ROS no avanza. Las ganancias de las tareas se mantienen sin cambios.
No envíes teleoperación u otras referencias a esa misma tarea mientras ejecuta.
Las tolerancias `tip_path` y `tip_goal` se comprueban por TF en los ejes congelados
del frame original del objetivo; `goal_time_tolerance` permite convergencia final.

El reloj ROS gobierna el avance: pausar simulación lo pausa y un salto hacia
atrás aborta el movimiento. Un TF perdido/obsoleto o exceso de error aborta el
seguimiento. Al terminar o cancelar se publica la pose medida, se anula el
feedforward y se restaura `auto_update_target_pose`; si se pierde TF se conserva
la última medida válida. Si falla la restauración, el resultado lo indica.
El envío por topics conserva la última referencia si desaparece el proceso del
planificador. Los modos **Controller** y **Task** mantienen su ejecución existente.

## Planificación con trayectorias de base y articulaciones

`PlanCartesianTrajectory` alcanza una pose de la pinza con el brazo seleccionado
y, cuando el goal lo permite, la base del AUV. La planificación, las colisiones
y la temporización son conjuntas; la ejecución reparte el resultado a
`BasePoseTrajectoryTask` y a la tarea articular existente.

## Arranque y planificación

Con el robot, TF y estados disponibles:

```bash
ros2 launch sura_manipulation_actions manipulation_actions.launch.py \
  execution_backend:=task use_rviz:=true use_rqt:=true
ros2 lifecycle set /plan_cartesian_trajectory_lifecycle_action_server configure
ros2 lifecycle set /plan_cartesian_trajectory_lifecycle_action_server activate
```

El launch comparte un SRDF con raíz `world_joint` floating y frame `world_ned`
entre los cuatro servidores y RViz. Los grupos `alpha_left`, `alpha_right` y
`dual_alpha` siguen sin comandar la base; los grupos `base_alpha_left` y
`base_alpha_right` permiten planificarla. El xacro conserva `floating_base:=false`
por defecto para otros consumidores; los launches de manipulación lo habilitan.

El launch no activa controladores. Para ejecutar con base, deben estar activos
`task_priority_controller` y `body_velocity`. El coordinador habilita la tarea de
base y aparta las tareas competidoras al admitir la ejecución.

## Goal y máscara de movilidad

| Campo | Significado |
| --- | --- |
| `arm` | Brazo configurado, por ejemplo `alpha_left`. |
| `target_pose` | Pose absoluta de la pinza en cualquier frame TF. Vacío selecciona `world_ned`; sello cero usa un instante común reciente. |
| `available_dofs` | `[x,y,z,roll,pitch,yaw,joints...]`. Vacío bloquea la base y permite todos los joints. |
| `base_limits` | Rangos de pose, velocidades y aceleraciones máximas por coordenada. |
| `tracking_tolerances` | Tolerancias de seguimiento/finales de base, joints y pinza, y tiempo adicional de llegada. |
| `position_tolerance`, `orientation_tolerance` | Restricciones geométricas cartesianas, distintas de las tolerancias de ejecución. |
| `cartesian_step`, `planning_time` | Resolución y presupuesto de planificación; no positivos usan configuración. |
| `velocity_scaling`, `acceleration_scaling` | Escalan la temporización conjunta; cero usa configuración. |
| `execution_backend` | `task` para base y brazo; `controller` admite brazo con base bloqueada. Vacío selecciona automáticamente `task` si se habilita la base. |
| `execute` | False permite revisar el plan antes de ejecutar. |

Los seis primeros booleanos siempre describen las coordenadas de la base en
`world_ned`, independientemente del frame del objetivo. La orientación usa RPY
ZYX de ROS. Un booleano falso fija la coordenada a su valor inicial en todo el
plan; no elimina restricciones del objetivo cartesiano.

El orden articular se publica en los parámetros de solo lectura
`arms.<arm>.joint_names`, derivados del modelo. Para Alpha es
`axis_b,axis_c,axis_d,axis_e`. No se toma el orden de `JointState`.

```bash
ros2 param get /plan_cartesian_trajectory_lifecycle_action_server arms.alpha_left.joint_names
```

El objetivo se transforma una vez al mundo y queda congelado, también cuando
la ejecución está pendiente. Las tolerancias cartesianas conservan los ejes del
frame de entrada, congelados al planificar. Un objetivo en `base_link` significa
un punto fijado en el mundo desde la pose inicial de la base.

## Límites y tolerancias

`base_limits.lower_limits` y `upper_limits` son arrays de seis componentes:
XYZ son offsets de la posición inicial, en ejes mundo, y roll/pitch/yaw son
ángulos absolutos respecto a `world_ned`. Deben aparecer juntos; vacíos no
añaden límites de pose. La pose inicial debe estar dentro de los rangos.
Yaw es continuo alrededor del inicio; intervalos que cruzan ±pi deben escribirse
como intervalos continuos, no con mínimo mayor que máximo. Pitch debe mantenerse
fuera de su singularidad.

`max_velocity` y `max_acceleration` admiten seis valores positivos. Vacíos usan
`planning.base_velocity_limits` y `planning.base_acceleration_limits`, inicialmente
`[1.0,1.0,1.0,0.5,0.5,0.5]` para velocidad y
`[0.4,0.4,0.4,0.2,0.2,0.2]` para aceleración, en m/s o rad/s y m/s² o rad/s².

Estos rangos no modifican ni se combinan con `stabilize_limits`. Esa tarea sigue
independiente y puede impedir seguir una trayectoria, provocando un fallo de
seguimiento. Se mantienen también límites articulares y autocolisiones.

Las tolerancias `base_path` y `base_goal` tienen seis valores XYZ/RPY;
`joint_path` y `joint_goal` siguen el orden articular, y `tip_path`/`tip_goal`
contienen XYZ y componentes del error log SO(3), en los ejes congelados del
objetivo. Arrays vacíos usan configuración; un cero explícito es estricto.
Los defaults son 0.05 m/0.3 rad para base, 0.05 rad por joint y
0.02 m/0.3 rad para pinza. `goal_time_tolerance` cero usa 0.5 s.

## Ejecución, cancelación y mantenimiento

El servidor mantiene su protocolo de plan pendiente:

```bash
ros2 service call /plan_cartesian_trajectory_lifecycle_action_server/execute_pending_trajectory std_srvs/srv/Trigger '{}'
ros2 service call /plan_cartesian_trajectory_lifecycle_action_server/cancel_pending_trajectory std_srvs/srv/Trigger '{}'
```

Antes de ejecutar se revalidan base, articulaciones y escena. Un plan obsoleto
exige otro goal; la preparación del backend no inicia el plan por sí sola.

El backend task envía una solicitud `FollowCartesianTrajectory` a
`/cirtesub/controller/task_priority/follow_cartesian_trajectory`. El controlador
reserva los recursos e instala ambas trayectorias con un sello inicial común y
un único reloj ROS. Pausar el tiempo de simulación pausa su progreso.

Durante el seguimiento se suspenden los holds de base/pinzas, la nivelación y
otras tareas competidoras. Las articulaciones del brazo no seleccionado se
mantienen medidas: su pinza acompaña a la base. Las órdenes incompatibles se
rechazan durante la ejecución.

El éxito mantiene el final, incluida la inclinación. Cancelar o fallar detiene
las referencias y mantiene el estado medido. Para volver a las tareas anteriores:

```bash
ros2 service call /cirtesub/controller/task_priority/release_cartesian_trajectory std_srvs/srv/Trigger '{}'
```

Los objetivos de mantenimiento se actualizan al estado medido; la estabilización
recupera su política anterior y puede volver a nivelar. Deactivar el controlador
termina el seguimiento y publica comandos cero.

## RQT, resultados y visualización

La pestaña **Cartesian** muestra casillas base/joints, límites por coordenada,
tolerancias y frame editable. **Read Current Pose (TF)** lee la pinza en el frame
seleccionado. Cambiar el frame transforma la pose escrita para conservar el
objetivo físico; si falta TF, se conserva el frame anterior. Habilitar una
coordenada base selecciona **Task** automáticamente.

El resultado conserva `joint_trajectory`, `path` y `desired_path`, y añade
`base_trajectory` y `base_path`. Los errores de seguimiento aparecen en feedback.
El botón de liberación está separado de cancelar el goal activo.

Los tópicos incluyen `/cirtesub/cartesian_planner/base_trajectory` y `base_path`,
además de `joint_trajectory`, `path` y `desired_path`. Paths y markers quedan en
el mundo. `DisplayTrajectory` incluye el estado floating; el visualizador común
publica el último plan en `/cirtesub/manipulation/markers`, con namespaces
`cartesian/base_path`, `cartesian/planned_path`, `cartesian/desired_path` y ejes.

### Objetivos interactivos independientes de RQT

El launch unificado arranca `planning_marker_server`, un proceso independiente con
executor propio. Puede servir los markers a RViz con RQT cerrado. La pestaña
Cartesian solo recibe su estado confirmado y envía ediciones/reservas mediante
`PlanningTargetState` y `UpdatePlanningTarget`, definidos en este paquete. No se
han cambiado las acciones de planificación ni las tablas de errores de RQT.

En RViz selecciona **Interact** y estos valores de **Interactive Markers Namespace**:

- Cartesiana: `/cirtesub/manipulation/cartesian_goal_markers`.
- Bimanual: `/cirtesub/manipulation/bimanual_cartesian_goal_markers`.

La propiedad es un namespace sin `/update`. Los archivos RViz suministrados ya
lo configuran correctamente. RQT muestra los servicios de edición en **Planning
target service**, por defecto `/cirtesub/manipulation/planning_targets/cartesian/update`
y `/cirtesub/manipulation/planning_targets/bimanual/update`. El estado está en
los tópicos hermanos `/state`, con QoS transient local para reconexión.

**Read Current Pose (TF)** pide la pose al nodo. Arrastrar una pinza actualiza RQT;
editar campos solicita una actualización al nodo. La cartesiana admite la máscara
existente de base y brazo. Las dos pinzas bimanuales definen solo `pB-pA` y
`RB*RAᵀ`, con base fija: sus dos poses absolutas no son restricciones del plan.

Antes de enviar un goal, RQT obtiene una reserva confirmada y el objetivo congelado.
La vista cartesiana queda fijada en `world_ned` con el mismo instante TF del goal,
conservando el frame de entrada para las tolerancias. El nodo sigue el UUID y el
estado de la acción, bloquea los markers durante el plan y libera la edición al
terminar o cancelar. Cerrar RQT no destruye el servidor. Si se pierde el editor y
no puede determinarse el goal activo, conserva la reserva y explica cómo liberarla
explícitamente tras cancelar la acción; no supone que el robot haya terminado.

También puede arrancarse por separado:

```bash
ros2 launch task_priority_kinematic_control_rqt planning_markers.launch.py
```

No arranques dos autoridades para el mismo robot: usa `launch_planning_markers:=false`
en el launch unificado si ya tienes una. Tras actualizar, cierra el RQT anterior
antes de relanzar para eliminar los servicios de la implementación antigua.
Con **Mostrar objetivos en RViz** desmarcado se mantiene la entrada manual sin
necesitar el nodo de markers. Las nuevas interfaces requieren regeneración mediante
la compilación del usuario.

En `config/example_cartesian_base_goal.yaml` hay un goal de ejemplo con XYZ y yaw
disponibles. Adapta el objetivo y los rangos a la pose medida antes de enviarlo.

Las interfaces deben regenerarse y sus clientes reconstruirse. Las pruebas C++
requieren compilación posterior del usuario; Codex no ejecuta builds sin petición
explícita. Las validaciones de interfaces, YAML, xacro y pruebas Python no necesitan
compilar C++.
