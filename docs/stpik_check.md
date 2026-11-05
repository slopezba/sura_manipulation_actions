# STPIK-check para Execute Pose

El goal de `PlanCartesianTrajectory` incluye `use_stpik` (por defecto `true`).
Con este campo activado, el backend `pose` solicita `CheckCartesianTrajectory` al
pulsar Execute Pending o con `execute: true`, antes de publicar cualquier
target/feedforward (incluido cero) o cambiar `auto_update_target_pose`.
Con `use_stpik=false` se omiten simulación y revalidación; se conservan disponibilidad,
TF y comprobación de tracking durante la ejecución. La planificación conserva su path cartesiano.
Task, Controller y las acciones bimanuales conservan su flujo.

La separación entre path cartesiano y comprobación con el controlador local sigue
el enfoque de [Di Lillo, Di Vito y Antonelli, Merging Global and Local Planners](https://iris.unicas.it/handle/11580/91662).
Se reutiliza el controlador efectivo de este workspace; no se añade RRT.

## Simulación aislada

`StpikTaskAccess::capture` copia bajo el mutex del host el estado de base y ambos
brazos, modelo, backend independiente, solver y tareas en su orden efectivo.
Incluye ganancias modificadas por servicios, referencias, máscaras, feedforward y
su instante de recepción, opciones de captura automática e histéresis set-based.
No reconstruye estos valores desde parámetros ROS.

Un worker ejecuta `StpikCheck::run` fuera del mutex, sin hardware ni publicadores:

1. Evalúa el mismo `CartesianPosePath` usado en ejecución, incluida duración y arco
   de cuaterniones. Instala pose y velocidad analítica en la copia de la tarea
   seleccionada, con captura automática desactivada.
2. Actualiza FK y Jacobianos del backend clonado.
3. Usa `TaskManager::update_tasks_into`, `HierarchySolver::solve` y
   `TaskManager::finalize_tasks_command`, compartidos con el controlador real.
4. Comprueba seguimiento, seguridad y comando final. Integra todos los DoF activos,
   usando Euler explícito para posiciones y `Exp(omega_world * dt) * R` para la
   orientación de base; las velocidades angulares no son derivadas de Euler.

Los timeouts de feedforward y trayectorias usan un reloj monotónico inyectable.
El worker lo avanza con tiempo simulado; no necesita que avance ROS time real.
La simulación es cinemática. La ejecución conserva sus comprobaciones por TF.

Se copian FramePose, EndEffectorPose/Position/Orientation, EndEffectorsRelativePose,
JointNominal, BaseYaw, JointLimits, StabilizeLimits y SelfCollisionAvoidance.
JointTrajectory/BasePoseTrajectory se admiten en reposo o manteniendo el último
punto; una trayectoria competidora devuelve `ACTIVE_TASK_BUSY`. Una tarea
habilitada no soportada, como EndEffectorPlanRelativePose con planificador
asíncrono, devuelve `UNSUPPORTED_ACTIVE_TASK`; nunca se omite silenciosamente.
Un backend sin clon independiente deja la comprobación no disponible.

## Parámetros

Los parámetros `stpik.*` pertenecen al nodo Task Priority y se leen al comenzar
cada check. Ambos ejemplos están en `task_priority_kinematic_control/config/`.

| Parámetro | Valor inicial | Significado |
| --- | --- | --- |
| `stpik.dt` | `0.01` s | Paso de integración; incluye exactamente el final y la convergencia. |
| `stpik.stall_window` | `1.0` s | Ventana de detección de atasco. |
| `stpik.min_position_progress` | `0.0001` m | Avance medido o reducción de error mínima. |
| `stpik.min_orientation_progress` | `0.001` rad | Avance angular o reducción de error mínima. |
| `stpik.numerical_tolerance` | `1e-6` | Margen numérico en restricciones/comparaciones. |
| `stpik.singular_value_tolerance` | `1e-6` | Diagnóstico del Jacobiano proyectado y ponderado. |
| `stpik.velocity_limits` | `[]` | Vacío usa límites efectivos; un vector completo añade límites más estrictos. |
| `stpik.wall_timeout` | `30.0` s | Presupuesto real; expirar deja viabilidad desconocida. |
| `stpik.snapshot_joint_tolerance` | `0.001` | Cambio articular máximo durante el check. |
| `stpik.snapshot_position_tolerance` | `0.005` m | Cambio de posición de base máximo. |
| `stpik.snapshot_orientation_tolerance` | `0.01` rad | Cambio angular de base máximo. |

El orden de velocidades es: ejes activos de base, articulaciones izquierdas y
derechas. Se usan los límites efectivos del controlador, sin sustituirlos por los
del URDF. La saturación normal del solver no constituye un fallo.

Las tolerancias cartesianas proceden exclusivamente de `tracking_tolerances.tip_path`
y `tracking_tolerances.tip_goal` del goal (los campos Tool Path/Goal Tracking Tolerance
en RQT), en sus ejes congelados. No hay un segundo límite de error en el controlador;
se eliminan `stpik.max_position_error` y `stpik.max_orientation_error`.
Si se dejan las tolerancias vacías, el servidor rellena sus valores por defecto
antes de enviar el check. Se exige `tip_goal` dentro de
`goal_time_tolerance` (por defecto 0.5 s). Límites articulares, inclinación y
distancias seguras proceden de las tareas activas del snapshot. La activación
preventiva de una tarea set-based es normal; rebasar su límite físico/seguro falla.

Se comprueban singularidades de la tarea seleccionada tras la proyección de
prioridades y ponderación. Un rango deficiente solo rechaza el path cuando impide
seguirlo, progresar o llegar. Las filas cero reservadas para restricciones
set-based inactivas no se cuentan. El reposo dentro de tolerancias es válido.

En el planificador, `pose.stpik_timeout` vale 35 s y debe cubrir worker y transporte.
`arms.<arm>.pose_stpik_action` configura la ruta, por defecto derivada de los topics
de la tarea. El perfil usa `/cirtesub/controller/task_priority/check_cartesian_trajectory`;
bringup remapea también sus cinco endpoints internos en Humble.

## Resultados y frescura

`SUCCESS` devuelve un identificador de validación. Al terminar, el host verifica
estado real, backend y configuración frente al snapshot. El planificador vuelve
a comprobar TF y disponibilidad y revalida ese identificador antes de preparar
la tarea. Cambios de ganancias, prioridades, habilitaciones, máscaras, referencias,
histéresis, solver, backend o parámetros del checker invalidan la comprobación.
El check no cambia ni bloquea la jerarquía real.
Si un feedforward externo con timeout seguía vigente al capturar, se requiere
reintentar tras su expiración: el tiempo invertido en validar no puede desplazar
su caducidad en la ejecución real.

El resultado incluye outcome/código/motivo, instante, nombres y posiciones
articulares, pose de base, error cartesiano, tarea/restricción, clearance mínimo,
máxima razón de velocidad y mínimo valor singular. La acción de planificación
transmite el diagnóstico en resultado y en feedback `stpik_failed`; `tip_error`
contiene el error simulado que provocó el rechazo. El nuevo campo `use_stpik`
requiere regenerar la interfaz de la acción y reiniciar sus clientes y servidores.

| Resultado | Ejemplos | Tratamiento |
| --- | --- | --- |
| `FAIL` | `PATH_TRACKING`, `JOINT_LIMIT`, `COLLISION_CLEARANCE`, `VELOCITY_LIMIT`, `STALLED`, `GOAL_TOLERANCE`, `UNSUPPORTED_ACTIVE_TASK` | Rechaza el plan. |
| `UNAVAILABLE` | `CHECK_TIMEOUT`, `STALE_SNAPSHOT`, checker ausente, host no preparado | Conserva el plan para reintentar Execute. |
| `CANCELED` | Cancelación o desactivación | Cancela sin enviar referencias. |

Los fallos de seguimiento asociados a rango usan `TRACKING_SINGULARITY`,
`STALLED_SINGULARITY` o `GOAL_SINGULARITY`. Geometría inválida y valores no finitos
se rechazan explícitamente. Los códigos quedan disponibles para futura replanning.

Los fallos de atasco incluyen `stalled=position`, `orientation` o
`position_and_orientation`, además de `position_stalled` y `orientation_stalled`.
El mensaje detalla la ventana temporal, movimiento solicitado y medido, reducción
del error y progreso mínimo configurado para posición (m) y orientación (rad).
Una orientación constante dentro de tolerancias aparece con `requested=false`
y `orientation_stalled=false`. Esta ampliación del diagnóstico conserva el criterio
de detección de atasco existente.

Las pruebas C++ cubren aislamiento, los tres solvers, seguridad, orientación,
saturación, atasco, convergencia, reloj simulado y cancelación. Las pruebas del
servidor cubren ambas entradas de ejecución, retención/reintento y revalidación.
La serialización comprueba pose y feedforward incluso con rotaciones de pi.
