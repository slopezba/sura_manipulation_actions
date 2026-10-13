# Planificador cartesiano de un brazo

`PlanCartesianTrajectory` planifica una trayectoria cartesiana de la pinza del
brazo elegido, desde la configuración medida hasta una pose absoluta expresada
en la base. Solo incluye las articulaciones de ese brazo en la `JointTrajectory`.
Reutiliza el solver restringido, FK/jacobianos, colisiones, suavizado y
temporización del planificador bimanual. No necesita una pareja o grupo conjunto.

## Un único launch

Con el robot, TF, joint states y controladores ya disponibles:

```bash
ros2 launch sura_manipulation_actions manipulation_actions.launch.py \
  execution_backend:=task use_rviz:=true use_rqt:=true
```

Este launch reúne por defecto cuatro servidores lifecycle:

| Acción | Nodo lifecycle |
| --- | --- |
| `plan_joint_trajectory` | `/plan_joint_trajectory_lifecycle_action_server` |
| `plan_bimanual_joint_trajectory` | `/plan_bimanual_joint_trajectory_lifecycle_action_server` |
| `plan_cartesian_trajectory` | `/plan_cartesian_trajectory_lifecycle_action_server` |
| `plan_bimanual_cartesian_trajectory` | `/plan_bimanual_cartesian_trajectory_lifecycle_action_server` |

Detén los launches anteriores de esos mismos servidores para evitar duplicados.
La activación sigue siendo manual. Para el nuevo servidor:

```bash
ros2 lifecycle set /plan_cartesian_trajectory_lifecycle_action_server configure
ros2 lifecycle set /plan_cartesian_trajectory_lifecycle_action_server activate
```

`launch_joint`, `launch_bimanual_joint`, `launch_cartesian` y
`launch_bimanual_cartesian` permiten deshabilitar servidores individuales.
`launch_hybrid:=true` incluye también el launch híbrido existente, opcional.
RViz y RQT son opcionales mediante `use_rviz` y `use_rqt`; el RQT requiere
`task_priority_kinematic_control_rqt` instalado en el mismo entorno.
No se arrancan ni activan controladores o tareas desde este launch.
Las acciones articulares conservan sus políticas existentes de ejecución;
la acción bimanual joint existente usa sus controladores de trayectoria.

## RQT e interfaz

La nueva pestaña **Cartesian** comparte el nodo y spinner del panel existente.
Elige **Arm**, **Execution Backend** y **Reference Frame**. Pulsa
**Read Current Pose (TF)** para leer la pose absoluta del tool elegido en la
base, edita XYZ/quaternion y pulsa **Send Goal**.

La acción es `/cirtesub/manipulation/plan_cartesian_trajectory`.

| Campo del goal | Significado |
| --- | --- |
| `arm` | Identificador de `arm_names`: `alpha_left` o `alpha_right` en el ejemplo. |
| `target_pose` | Pose absoluta de `standard_jaws_tool`; `frame_id` debe coincidir con `reference_frame`, por defecto `cirtesub/base_link`. |
| `position_tolerance` | Tolerancias X/Y/Z en metros; cero es una restricción estricta. |
| `orientation_tolerance` | Tolerancias por componente del error log SO(3), en radianes, en los ejes de la referencia. |
| `cartesian_step`, `planning_time` | Resolución lineal y presupuesto de cálculo; no positivos usan los defaults. |
| `velocity_scaling`, `acceleration_scaling` | Factores articulares `(0,1]`; cero usa el default del servidor. |
| `execution_backend` | `controller`, `task` o vacío para el default del launch. |
| `execute` | Por defecto false: revisar el plan y ejecutar mediante servicio. |

El resultado contiene éxito, diagnóstico, `JointTrajectory`, `path` alcanzado
por FK y `desired_path`. El feedback muestra progreso, etapa y diagnóstico.
La posición deseada se interpola en línea recta y la orientación con SLERP.
Un brazo con menos de seis grados de libertad puede no alcanzar una pose
arbitraria con orientación estricta; las tolerancias son parte del objetivo.

## Backend y plan pendiente

Los endpoints del ejemplo son:

| Brazo | `task` | `controller` |
| --- | --- | --- |
| `alpha_left` | `/cirtesub/controller/task_priority/tasks/left_joint_trajectory/follow_joint_trajectory` | `/cirtesub/alpha/controller/alpha_left_joint_trajectory_controller/follow_joint_trajectory` |
| `alpha_right` | `/cirtesub/controller/task_priority/tasks/right_joint_trajectory/follow_joint_trajectory` | `/cirtesub/alpha/controller/alpha_right_joint_trajectory_controller/follow_joint_trajectory` |

Se envía un único goal con las articulaciones seleccionadas. `task` usa stamp
cero; `controller` usa el inicio futuro del transporte común. El operador debe
activar el backend y, para `task`, habilitar la tarea articular elegida y
deshabilitar las tareas incompatibles sobre esas articulaciones, como la pose
del mismo brazo o la tarea bimanual. Para reproducir q/v/a mediante Task Priority,
configura `interpolation: quintic` en la tarea izquierda/derecha. El valor
predeterminado `linear` de esas tareas existentes se conserva.

**Execute Pending** llama a:

```bash
ros2 service call \
  /plan_cartesian_trajectory_lifecycle_action_server/execute_pending_trajectory \
  std_srvs/srv/Trigger '{}'
```

También existe `cancel_pending_trajectory`; el botón **Cancel Goal** cancela el
goal de la pestaña. Un backend inactivo o una tarea que rechace la admisión
conserva el plan pendiente. Activar el backend no ejecuta por sí solo: solicita
la ejecución otra vez. Antes del despacho se comprueban el estado inicial, la
base y la escena; un plan obsoleto exige volver a planificar.

## Configuración y RViz

`cartesian_planner_config_file` selecciona
`config/single_arm_cartesian_planner.yaml`. Cada brazo declara grupo MoveIt,
efector, endpoint de controlador y endpoint de tarea. Los límites y tamaños
articulares se obtienen del modelo. La referencia debe ser un link independiente
de las articulaciones comandadas. Se pueden configurar otros brazos o un único
brazo cambiando el YAML, URDF/SRDF, límites y kinematics del launch común.

Los tópicos del nuevo planificador son:

- `/cirtesub/cartesian_planner/joint_trajectory`.
- `/cirtesub/cartesian_planner/path` y `/cirtesub/cartesian_planner/desired_path`.
- `/cirtesub/manipulation/markers`, un único `MarkerArray` común con las demás
  acciones: ejes del objetivo y de la pose final (X rojo, Y verde, Z azul), línea
  planificada verde y deseada violeta. La línea deseada es algo más ancha y
  translúcida para poder ver ambas cuando coinciden.

Se publica además `DisplayTrajectory` completo y la escena en los tópicos
MoveIt existentes. Los paths y markers usan la base existente, sin TF nuevos;
los paths se refrescan a 10 Hz con timestamp cero y los markers son frame-locked.
La configuración RViz del launch común muestra planes individuales y bimanuales.
Usa un único display `MarkerArray` en `/cirtesub/manipulation/markers`.
Cada nuevo plan publica juntos los ejes y las líneas habilitados, en un solo
mensaje retenido mediante `transient_local`.
Los namespaces `cartesian/target` y `cartesian/final` distinguen el objetivo
(ejes de 6 cm) de la pose final calculada por FK (4 cm). Las líneas están en
`cartesian/planned_path` y `cartesian/desired_path`; se pueden ocultar por
namespace. Bimanual usa `bimanual_cartesian/...` en ese mismo tópico.
Un plan nuevo de cualquier acción sustituye todas las capas anteriores.
El publicador común `manipulation_marker_visualization`, iniciado por el launch,
recibe los planes en `/cirtesub/manipulation/marker_plans` y publica un solo
mensaje con `DELETEALL` seguido de los ejes y paths del último plan. El borrado
usa namespace vacío para borrar globalmente y un ID reservado para que cada
pareja namespace/ID aparezca una sola vez.
Solo este nodo retiene la salida (`transient_local`, profundidad 1), de modo
que abrir RViz después muestra únicamente el plan más reciente. Los
`nav_msgs/Path` siguen en sus tópicos individuales
para no intercalar datos de planes distintos durante el refresco a 10 Hz.
Las líneas del tópico común son frame-locked y acompañan al TF de la base
sin necesitar ese refresco. El tópico común usa QoS fiable
`transient_local`.

Usa un único nodo de visualización por robot. Si ya está activo, el launch
admite `launch_marker_visualization:=false`. Debe estar activo al planificar:
la entrada es volátil y reiniciar solo ese nodo requiere un nuevo plan.
Reabrir RViz conserva la última visualización mientras el nodo sigue activo.

Las pruebas Python de RQT y los checks de launch pueden ejecutarse sin compilar.
Se han añadido regresiones C++ para articulaciones variables, brazo no comandado
fijo en el plan, colisiones entre puntos, rutas controller/task, plan pendiente,
tarea deshabilitada, cancelación y estado inicial obsoleto. Estas pruebas C++
quedan pendientes de compilación y ejecución por el usuario.
