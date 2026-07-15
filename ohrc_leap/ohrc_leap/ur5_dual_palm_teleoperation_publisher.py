#!/usr/bin/env python3
"""
UR5 Dual Palm Teleoperation Publisher
Converts Leap Motion palm data to State commands for bimanual UR5 teleoperation.
Uses palm center position and orientation for control.
"""

from hand_msgs.msg import Hands, Hand
from rclpy.node import Node
import rclpy
from ohrc_msgs.msg import State
from geometry_msgs.msg import Quaternion
import math


class UR5DualPalmTeleoperationPublisher(Node):
    def __init__(self):
        super().__init__('ur5_dual_palm_teleoperation_publisher')
        
        # Subscribe to Leap Motion hands data
        self.hands_subscriber = self.create_subscription( 
            Hands,
            'leap_hands',
            self.hands_callback,
            10
        )

        # Publishers for each UR5 arm (using namespaces)
        self.left_state_publisher = self.create_publisher(State, '/ur5_left/cmd_state', 10)
        self.right_state_publisher = self.create_publisher(State, '/ur5_right/cmd_state', 10)

        # Parameters for control sensitivity
        self.declare_parameter('position_scale', 1.0)
        self.declare_parameter('velocity_scale', 1.0)
        self.declare_parameter('grab_threshold', 0.7)  # Threshold to enable/disable control
        self.declare_parameter('max_delta_position', 0.5)  # Maximum delta position in meters (prevents jumps)
        # Nuevo: parámetro global para habilitar / deshabilitar teleoperación
        # Así no necesitas mantener los puños cerrados todo el tiempo.
        self.declare_parameter('teleop_enabled', False)
        
        # Parameters for orientation control (invert roll, pitch, yaw)
        self.declare_parameter('invert_roll', False)   # Invertir rotación alrededor de X
        self.declare_parameter('invert_pitch', False)  # Invertir rotación alrededor de Y
        self.declare_parameter('invert_yaw', False)    # Invertir rotación alrededor de Z (muñeca)
        
        # Parameters for initial pose (from ur5_hw_config.yaml)
        # Format: [x, y, z, roll, pitch, yaw]
        self.declare_parameter('left_initial_pose', [0.55, 0.25, 1.2, 3.141592, 1.5, 1.8])
        self.declare_parameter('right_initial_pose', [0.55, -0.25, 1.2, 3.141592, 1.5, -1.8])
        
        self.position_scale = self.get_parameter('position_scale').value
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.grab_threshold = self.get_parameter('grab_threshold').value
        self.max_delta_position = self.get_parameter('max_delta_position').value
        self.teleop_enabled = self.get_parameter('teleop_enabled').value
        self.invert_roll = self.get_parameter('invert_roll').value
        self.invert_pitch = self.get_parameter('invert_pitch').value
        self.invert_yaw = self.get_parameter('invert_yaw').value
        
        # Callback para actualizar poses cuando cambian los parámetros
        self.add_on_set_parameters_callback(self._on_parameters_changed)
        
        # Cargar poses iniciales
        self._update_initial_poses()
        
        # Simple initialization: cuando se detectan manos por primera vez,
        # esperamos un poco antes de activar control para usar la pose inicial
        self.left_hand_first_detection = True
        self.right_hand_first_detection = True
        self.left_init_counter = 0
        self.right_init_counter = 0
        self.init_delay_callbacks = 15  # Callbacks a esperar antes de activar control
        
        # Flags para enviar posición cero en el primer comando habilitado
        # Esto asegura que T_state_start = (0,0,0) en el código C++
        self.left_first_enabled_sent = False
        self.right_first_enabled_sent = False
        
        # Guardar posición inicial de la mano cuando se activa teleoperación
        # Esto permite calcular movimientos relativos desde el punto de activación
        self.left_hand_initial_position = None  # [x, y, z] de la mano cuando se activó
        self.left_hand_initial_orientation = None  # Quaternion de la mano cuando se activó
        self.right_hand_initial_position = None
        self.right_hand_initial_orientation = None
        
        self.get_logger().info('UR5 Dual Palm Teleoperation Publisher initialized')
        self.get_logger().info(f'Position scale: {self.position_scale}')
        self.get_logger().info(f'Velocity scale: {self.velocity_scale}')
        self.get_logger().info(f'Grab threshold: {self.grab_threshold}')
        self.get_logger().info(f'Max delta position: {self.max_delta_position} m (límite para prevenir saltos)')
        self.get_logger().info(f'Teleop enabled (parámetro inicial): {self.teleop_enabled}')
        self.get_logger().info(f'Invert Roll: {self.invert_roll}, Pitch: {self.invert_pitch}, Yaw: {self.invert_yaw}')


    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """
        Convert Euler angles (roll, pitch, yaw) to quaternion.
        
        Args:
            roll: Rotation around X axis (radians)
            pitch: Rotation around Y axis (radians)
            yaw: Rotation around Z axis (radians)
        
        Returns:
            Quaternion message
        """
        # Half angles
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q

    def _multiply_quaternions(self, q1: Quaternion, q2: Quaternion) -> Quaternion:
        """
        Multiply two quaternions: result = q1 * q2
        This combines rotations: first q1, then q2.
        
        Args:
            q1: First quaternion (w, x, y, z)
            q2: Second quaternion (w, x, y, z)
        
        Returns:
            Combined quaternion
        """
        result = Quaternion()
        result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        
        # Normalize the quaternion
        norm = math.sqrt(result.w**2 + result.x**2 + result.y**2 + result.z**2)
        if norm > 0.0:
            result.w /= norm
            result.x /= norm
            result.y /= norm
            result.z /= norm
        
        return result

    def _quaternion_inverse(self, q: Quaternion) -> Quaternion:
        """
        Calculate the inverse of a quaternion.
        For a normalized quaternion, the inverse is (w, -x, -y, -z).
        
        Args:
            q: Input quaternion (w, x, y, z)
        
        Returns:
            Inverse quaternion
        """
        result = Quaternion()
        # For normalized quaternion, inverse is conjugate
        result.w = q.w
        result.x = -q.x
        result.y = -q.y
        result.z = -q.z
        return result

    def _update_initial_poses(self):
        """
        Actualizar las poses iniciales desde los parámetros.
        Se llama cuando cambian los parámetros left_initial_pose o right_initial_pose.
        """
        left_pose_raw = self.get_parameter('left_initial_pose').value
        right_pose_raw = self.get_parameter('right_initial_pose').value
        
        # Store initial poses: [x, y, z] and quaternion
        self.left_initial_position = [left_pose_raw[0], left_pose_raw[1], left_pose_raw[2]]
        self.right_initial_position = [right_pose_raw[0], right_pose_raw[1], right_pose_raw[2]]
        
        # Convert roll, pitch, yaw to quaternion
        self.left_initial_orientation = self._euler_to_quaternion(
            left_pose_raw[3], left_pose_raw[4], left_pose_raw[5]
        )
        self.right_initial_orientation = self._euler_to_quaternion(
            right_pose_raw[3], right_pose_raw[4], right_pose_raw[5]
        )
        
        self.get_logger().info(f'🔄 Poses iniciales actualizadas')
        self.get_logger().info(f'   LEFT: pos={self.left_initial_position}, rpy={left_pose_raw[3:6]}')
        self.get_logger().info(f'   RIGHT: pos={self.right_initial_position}, rpy={right_pose_raw[3:6]}')
    
    def _get_bool_value(self, param_value):
        """
        Helper para obtener el valor booleano de un parámetro.
        Maneja tanto el caso donde param_value es un bool directamente
        como cuando es un objeto ParameterValue con bool_value.
        """
        if isinstance(param_value, bool):
            return param_value
        elif hasattr(param_value, 'bool_value'):
            return param_value.bool_value
        else:
            # Fallback: intentar convertir a bool
            return bool(param_value)
    
    def _on_parameters_changed(self, params):
        """
        Callback cuando cambian los parámetros.
        Permite actualizar las poses iniciales sin reiniciar el nodo.
        """
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            if param.name in ['left_initial_pose', 'right_initial_pose']:
                self._update_initial_poses()
                # Reset flags para que se use la nueva pose inicial en la próxima activación
                self.left_first_enabled_sent = False
                self.right_first_enabled_sent = False
                self.left_hand_initial_position = None
                self.left_hand_initial_orientation = None
                self.right_hand_initial_position = None
                self.right_hand_initial_orientation = None
                self.get_logger().info(f'✅ Parámetro {param.name} actualizado. La nueva pose se usará en la próxima activación.')
            elif param.name == 'teleop_enabled':
                # Activar / desactivar teleoperación globalmente
                self.teleop_enabled = self._get_bool_value(param.value)
                estado = "ACTIVADA" if self.teleop_enabled else "DESACTIVADA"
                self.get_logger().info(f'✅ Teleoperación {estado} mediante parámetro teleop_enabled')
                # Cuando cambiamos el estado global, forzamos a que la próxima activación
                # vuelva a capturar la posición inicial de la mano
                self.left_first_enabled_sent = False
                self.right_first_enabled_sent = False
                self.left_hand_initial_position = None
                self.left_hand_initial_orientation = None
                self.right_hand_initial_position = None
                self.right_hand_initial_orientation = None
            elif param.name in ['invert_roll', 'invert_pitch', 'invert_yaw']:
                # Actualizar valores de inversión
                bool_val = self._get_bool_value(param.value)
                if param.name == 'invert_roll':
                    self.invert_roll = bool_val
                elif param.name == 'invert_pitch':
                    self.invert_pitch = bool_val
                elif param.name == 'invert_yaw':
                    self.invert_yaw = bool_val
                self.get_logger().info(f'⚠️ Parámetro {param.name} cambió. IMPORTANTE: Debes reactivar la teleoperación (quitar y volver a poner las manos) para que el cambio tenga efecto.')
                # Reset flags para forzar reactivación
                self.left_first_enabled_sent = False
                self.right_first_enabled_sent = False
                self.left_hand_initial_position = None
                self.left_hand_initial_orientation = None
                self.right_hand_initial_position = None
                self.right_hand_initial_orientation = None
        
        result = SetParametersResult()
        result.successful = True
        return result
    
    def _get_rpy_from_quaternion(self, q: Quaternion):
        """Helper para convertir quaternion a RPY para logging"""
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]

    def _print_pose(self, arm_name: str, state_cmd: State):
        """
        Print the current pose (position and orientation) being sent to an arm.
        
        Args:
            arm_name: Name of the arm ("LEFT" or "RIGHT")
            state_cmd: State message containing the pose
        """
        # Convert quaternion to Euler angles for readability
        q = state_cmd.pose.orientation
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Print pose information
        self.get_logger().info(
            f'🤖 {arm_name} ARM - Pose: '
            f'pos=({state_cmd.pose.position.x:.3f}, {state_cmd.pose.position.y:.3f}, {state_cmd.pose.position.z:.3f}) '
            f'rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f}) '
            f'enabled={state_cmd.enabled}'
        )

    def _reset_state_to_zero(self, state_cmd: State):
        """
        Reset State command to zero (safe state when no hands detected).
        
        Args:
            state_cmd: State message to reset
        """
        state_cmd.enabled = False
        # Reset position
        state_cmd.pose.position.x = 0.0
        state_cmd.pose.position.y = 0.0
        state_cmd.pose.position.z = 0.0
        # Reset orientation to identity quaternion (no rotation)
        state_cmd.pose.orientation.w = 1.0
        state_cmd.pose.orientation.x = 0.0
        state_cmd.pose.orientation.y = 0.0
        state_cmd.pose.orientation.z = 0.0
        # Reset linear velocity
        state_cmd.twist.linear.x = 0.0
        state_cmd.twist.linear.y = 0.0
        state_cmd.twist.linear.z = 0.0
        # Reset angular velocity
        state_cmd.twist.angular.x = 0.0
        state_cmd.twist.angular.y = 0.0
        state_cmd.twist.angular.z = 0.0
        state_cmd.reset = False


    def hands_callback(self, msg):
        """
        Process Leap Motion hands data and publish State commands for both UR5 arms.
        
        Control scheme:
        - Right hand palm position/orientation -> UR5 right arm
        - Left hand palm position/orientation -> UR5 left arm
        - Grab strength > threshold -> enable control
        - Grab strength < threshold -> disable control
        """
        
        # Initialize commands for both arms
        left_cmd = State()
        right_cmd = State()
        
        # Check if we have hands detected
        if len(msg.hands) == 0:
            # No hands detected - disable both arms and return to initial pose (SEGURO)
            # Reset first detection flags so next time hands are detected, we use initial pose
            self.left_hand_first_detection = True
            self.right_hand_first_detection = True
            self.left_init_counter = 0
            self.right_init_counter = 0
            self.left_first_enabled_sent = False  # Reset flag
            self.right_first_enabled_sent = False  # Reset flag
            self.left_hand_initial_position = None  # Reset posición inicial de la mano
            self.left_hand_initial_orientation = None
            self.right_hand_initial_position = None
            self.right_hand_initial_orientation = None
            # Enviar pose inicial del YAML en lugar de (0,0,0)
            left_cmd.pose.position.x = self.left_initial_position[0]
            left_cmd.pose.position.y = self.left_initial_position[1]
            left_cmd.pose.position.z = self.left_initial_position[2]
            left_cmd.pose.orientation = self.left_initial_orientation
            left_cmd.twist.linear.x = 0.0
            left_cmd.twist.linear.y = 0.0
            left_cmd.twist.linear.z = 0.0
            left_cmd.twist.angular.x = 0.0
            left_cmd.twist.angular.y = 0.0
            left_cmd.twist.angular.z = 0.0
            left_cmd.enabled = False
            left_cmd.reset = False
            
            right_cmd.pose.position.x = self.right_initial_position[0]
            right_cmd.pose.position.y = self.right_initial_position[1]
            right_cmd.pose.position.z = self.right_initial_position[2]
            right_cmd.pose.orientation = self.right_initial_orientation
            right_cmd.twist.linear.x = 0.0
            right_cmd.twist.linear.y = 0.0
            right_cmd.twist.linear.z = 0.0
            right_cmd.twist.angular.x = 0.0
            right_cmd.twist.angular.y = 0.0
            right_cmd.twist.angular.z = 0.0
            right_cmd.enabled = False
            right_cmd.reset = False
            
            self._print_pose("LEFT", left_cmd)
            self._print_pose("RIGHT", right_cmd)
            self.left_state_publisher.publish(left_cmd)
            self.right_state_publisher.publish(right_cmd)
            return
        
        # Process each detected hand
        left_hand_detected = False
        right_hand_detected = False
        left_hand = None
        right_hand = None
        
        for hand in msg.hands:
            if hand.type == "Left":
                left_hand_detected = True
                left_hand = hand
                
            elif hand.type == "Right":
                right_hand_detected = True
                right_hand = hand
        
        # Publish commands
        if left_hand_detected:
            # Cuando se detecta la mano por primera vez, esperamos antes de activar control
            # Esto permite que el sistema use la pose inicial como referencia
            if self.left_hand_first_detection:
                if self.left_init_counter < self.init_delay_callbacks:
                    # Enviamos comandos deshabilitados con posición cero durante la inicialización
                    if self.left_init_counter == 0:
                        self.get_logger().info(f'🔵 LEFT: Iniciando inicialización (contador: {self.left_init_counter}/{self.init_delay_callbacks})')
                    init_cmd = State()
                    init_cmd.enabled = False  # Mantener deshabilitado durante inicialización
                    init_cmd.pose.position.x = 0.0
                    init_cmd.pose.position.y = 0.0
                    init_cmd.pose.position.z = 0.0
                    init_cmd.pose.orientation.w = 1.0
                    init_cmd.pose.orientation.x = 0.0
                    init_cmd.pose.orientation.y = 0.0
                    init_cmd.pose.orientation.z = 0.0
                    init_cmd.twist.linear.x = 0.0
                    init_cmd.twist.linear.y = 0.0
                    init_cmd.twist.linear.z = 0.0
                    init_cmd.twist.angular.x = 0.0
                    init_cmd.twist.angular.y = 0.0
                    init_cmd.twist.angular.z = 0.0
                    init_cmd.reset = False
                    self._print_pose("LEFT", init_cmd)
                    self.left_state_publisher.publish(init_cmd)
                    self.left_init_counter += 1
                    # NO procesar la mano todavía, solo enviar comando en cero
                    # Continuar para procesar la mano derecha también
                else:
                    # Inicialización completa, ahora activamos control
                    self.get_logger().info(f'🟢 LEFT: Inicialización completa, activando control')
                    self.left_hand_first_detection = False
                    self.left_init_counter = 0
                    
                    # Verificar si la teleoperación global está habilitada
                    if not self.teleop_enabled:
                        # Teleoperación DESACTIVADA: mantener brazo en pose inicial, sin control
                        left_cmd.pose.position.x = self.left_initial_position[0]
                        left_cmd.pose.position.y = self.left_initial_position[1]
                        left_cmd.pose.position.z = self.left_initial_position[2]
                        left_cmd.pose.orientation = self.left_initial_orientation
                        left_cmd.twist.linear.x = 0.0
                        left_cmd.twist.linear.y = 0.0
                        left_cmd.twist.linear.z = 0.0
                        left_cmd.twist.angular.x = 0.0
                        left_cmd.twist.angular.y = 0.0
                        left_cmd.twist.angular.z = 0.0
                        left_cmd.enabled = False
                        left_cmd.reset = False
                        self._print_pose("LEFT", left_cmd)
                        self.left_state_publisher.publish(left_cmd)
                    else:
                        # Teleoperación ACTIVADA: usar mano como referencia
                        # PRIMER comando habilitado: guardar posición inicial de la mano y enviar pose inicial del robot
                        if not self.left_first_enabled_sent:
                            self.get_logger().info(f'📍 LEFT: Activando teleoperación - guardando pose inicial de la mano')
                            # Guardar posición inicial de la mano (con escalado e inversiones)
                            # IMPORTANTE: Aplicar las mismas transformaciones que en _process_hand_to_state
                            # Esto asegura que el frame de referencia sea consistente
                            hand_x = -left_hand.position.x * self.position_scale
                            hand_y = -left_hand.position.y * self.position_scale
                            hand_z = left_hand.position.z * self.position_scale
                            self.left_hand_initial_position = [hand_x, hand_y, hand_z]
                            self.get_logger().info(f'📍 LEFT: Mano inicial guardada: pos=({hand_x:.3f}, {hand_y:.3f}, {hand_z:.3f})')
                            
                            # Guardar orientación inicial de la mano (con inversiones si aplica)
                            if self.invert_roll or self.invert_pitch or self.invert_yaw:
                                hand_orient = Quaternion()
                                hand_orient.w = left_hand.orientation.w
                                hand_orient.x = -left_hand.orientation.x if self.invert_roll else left_hand.orientation.x
                                hand_orient.y = -left_hand.orientation.y if self.invert_pitch else left_hand.orientation.y
                                hand_orient.z = -left_hand.orientation.z if self.invert_yaw else left_hand.orientation.z
                                self.left_hand_initial_orientation = hand_orient
                            else:
                                self.left_hand_initial_orientation = left_hand.orientation
                            
                            # Enviar pose inicial del robot (del YAML)
                            left_cmd.pose.position.x = self.left_initial_position[0]
                            left_cmd.pose.position.y = self.left_initial_position[1]
                            left_cmd.pose.position.z = self.left_initial_position[2]
                            left_cmd.pose.orientation = self.left_initial_orientation
                            left_cmd.twist.linear.x = 0.0
                            left_cmd.twist.linear.y = 0.0
                            left_cmd.twist.linear.z = 0.0
                            left_cmd.twist.angular.x = 0.0
                            left_cmd.twist.angular.y = 0.0
                            left_cmd.twist.angular.z = 0.0
                            left_cmd.enabled = True  # Activar control
                            left_cmd.reset = False
                            self._print_pose("LEFT", left_cmd)
                            self.left_state_publisher.publish(left_cmd)
                            self.left_first_enabled_sent = True
                        else:
                            # Ya se envió el primer comando, procesar la mano normalmente (movimientos relativos)
                            self._process_hand_to_state(left_hand, left_cmd, is_left=True)
                            self._print_pose("LEFT", left_cmd)
                            self.left_state_publisher.publish(left_cmd)
            else:
                # No es la primera detección
                if left_hand:
                    if self.teleop_enabled:
                        # Teleoperación activa: seguir la mano normalmente
                        self._process_hand_to_state(left_hand, left_cmd, is_left=True)
                        self._print_pose("LEFT", left_cmd)
                        self.left_state_publisher.publish(left_cmd)
                    else:
                        # Teleoperación desactivada: mantener pose inicial, sin control
                        left_cmd.pose.position.x = self.left_initial_position[0]
                        left_cmd.pose.position.y = self.left_initial_position[1]
                        left_cmd.pose.position.z = self.left_initial_position[2]
                        left_cmd.pose.orientation = self.left_initial_orientation
                        left_cmd.twist.linear.x = 0.0
                        left_cmd.twist.linear.y = 0.0
                        left_cmd.twist.linear.z = 0.0
                        left_cmd.twist.angular.x = 0.0
                        left_cmd.twist.angular.y = 0.0
                        left_cmd.twist.angular.z = 0.0
                        left_cmd.enabled = False
                        left_cmd.reset = False
                        self._print_pose("LEFT", left_cmd)
                        self.left_state_publisher.publish(left_cmd)
        else:
            # No se detecta mano izquierda - deshabilitar brazo izquierdo
            # En lugar de resetear a (0,0,0), mantener la pose inicial del YAML
            self.left_hand_first_detection = True
            self.left_init_counter = 0
            self.left_first_enabled_sent = False  # Reset flag cuando se pierde la mano
            self.left_hand_initial_position = None  # Reset posición inicial de la mano
            self.left_hand_initial_orientation = None  # Reset orientación inicial de la mano
            # Enviar pose inicial del YAML en lugar de (0,0,0)
            left_cmd.pose.position.x = self.left_initial_position[0]
            left_cmd.pose.position.y = self.left_initial_position[1]
            left_cmd.pose.position.z = self.left_initial_position[2]
            left_cmd.pose.orientation = self.left_initial_orientation
            left_cmd.twist.linear.x = 0.0
            left_cmd.twist.linear.y = 0.0
            left_cmd.twist.linear.z = 0.0
            left_cmd.twist.angular.x = 0.0
            left_cmd.twist.angular.y = 0.0
            left_cmd.twist.angular.z = 0.0
            left_cmd.enabled = False  # Deshabilitar control
            left_cmd.reset = False
            self._print_pose("LEFT", left_cmd)
            self.left_state_publisher.publish(left_cmd)
            
        if right_hand_detected:
            # Cuando se detecta la mano por primera vez, esperamos antes de activar control
            # Esto permite que el sistema use la pose inicial como referencia
            if self.right_hand_first_detection:
                if self.right_init_counter < self.init_delay_callbacks:
                    # Enviamos comandos deshabilitados con posición cero durante la inicialización
                    if self.right_init_counter == 0:
                        self.get_logger().info(f'🔵 RIGHT: Iniciando inicialización (contador: {self.right_init_counter}/{self.init_delay_callbacks})')
                    init_cmd = State()
                    init_cmd.enabled = False  # Mantener deshabilitado durante inicialización
                    init_cmd.pose.position.x = 0.0
                    init_cmd.pose.position.y = 0.0
                    init_cmd.pose.position.z = 0.0
                    init_cmd.pose.orientation.w = 1.0
                    init_cmd.pose.orientation.x = 0.0
                    init_cmd.pose.orientation.y = 0.0
                    init_cmd.pose.orientation.z = 0.0
                    init_cmd.twist.linear.x = 0.0
                    init_cmd.twist.linear.y = 0.0
                    init_cmd.twist.linear.z = 0.0
                    init_cmd.twist.angular.x = 0.0
                    init_cmd.twist.angular.y = 0.0
                    init_cmd.twist.angular.z = 0.0
                    init_cmd.reset = False
                    self._print_pose("RIGHT", init_cmd)
                    self.right_state_publisher.publish(init_cmd)
                    self.right_init_counter += 1
                    # NO procesar la mano todavía, solo enviar comando en cero
                else:
                    # Inicialización completa, ahora activamos control
                    self.get_logger().info(f'🟢 RIGHT: Inicialización completa, activando control')
                    self.right_hand_first_detection = False
                    self.right_init_counter = 0
                    
                    # Verificar si la teleoperación global está habilitada
                    if not self.teleop_enabled:
                        # Teleoperación DESACTIVADA: mantener brazo en pose inicial, sin control
                        right_cmd.pose.position.x = self.right_initial_position[0]
                        right_cmd.pose.position.y = self.right_initial_position[1]
                        right_cmd.pose.position.z = self.right_initial_position[2]
                        right_cmd.pose.orientation = self.right_initial_orientation
                        right_cmd.twist.linear.x = 0.0
                        right_cmd.twist.linear.y = 0.0
                        right_cmd.twist.linear.z = 0.0
                        right_cmd.twist.angular.x = 0.0
                        right_cmd.twist.angular.y = 0.0
                        right_cmd.twist.angular.z = 0.0
                        right_cmd.enabled = False
                        right_cmd.reset = False
                        self._print_pose("RIGHT", right_cmd)
                        self.right_state_publisher.publish(right_cmd)
                    else:
                        # Teleoperación ACTIVADA: usar mano como referencia
                        # PRIMER comando habilitado: guardar posición inicial de la mano y enviar pose inicial del robot
                        if not self.right_first_enabled_sent:
                            self.get_logger().info(f'📍 RIGHT: Activando teleoperación - guardando pose inicial de la mano')
                            # Guardar posición inicial de la mano (con escalado e inversiones)
                            # IMPORTANTE: Aplicar las mismas transformaciones que en _process_hand_to_state
                            # Esto asegura que el frame de referencia sea consistente
                            hand_x = -right_hand.position.x * self.position_scale
                            hand_y = -right_hand.position.y * self.position_scale
                            hand_z = right_hand.position.z * self.position_scale
                            self.right_hand_initial_position = [hand_x, hand_y, hand_z]
                            self.get_logger().info(f'📍 RIGHT: Mano inicial guardada: pos=({hand_x:.3f}, {hand_y:.3f}, {hand_z:.3f})')
                            
                            # Guardar orientación inicial de la mano (con inversiones si aplica)
                            if self.invert_roll or self.invert_pitch or self.invert_yaw:
                                hand_orient = Quaternion()
                                hand_orient.w = right_hand.orientation.w
                                hand_orient.x = -right_hand.orientation.x if self.invert_roll else right_hand.orientation.x
                                hand_orient.y = -right_hand.orientation.y if self.invert_pitch else right_hand.orientation.y
                                hand_orient.z = -right_hand.orientation.z if self.invert_yaw else right_hand.orientation.z
                                self.right_hand_initial_orientation = hand_orient
                            else:
                                self.right_hand_initial_orientation = right_hand.orientation
                            
                            # Enviar pose inicial del robot (del YAML)
                            right_cmd.pose.position.x = self.right_initial_position[0]
                            right_cmd.pose.position.y = self.right_initial_position[1]
                            right_cmd.pose.position.z = self.right_initial_position[2]
                            right_cmd.pose.orientation = self.right_initial_orientation
                            right_cmd.twist.linear.x = 0.0
                            right_cmd.twist.linear.y = 0.0
                            right_cmd.twist.linear.z = 0.0
                            right_cmd.twist.angular.x = 0.0
                            right_cmd.twist.angular.y = 0.0
                            right_cmd.twist.angular.z = 0.0
                            right_cmd.enabled = True  # Activar control
                            right_cmd.reset = False
                            self._print_pose("RIGHT", right_cmd)
                            self.right_state_publisher.publish(right_cmd)
                            self.right_first_enabled_sent = True
                        else:
                            # Ya se envió el primer comando, procesar la mano normalmente (movimientos relativos)
                            self._process_hand_to_state(right_hand, right_cmd, is_left=False)
                            self._print_pose("RIGHT", right_cmd)
                            self.right_state_publisher.publish(right_cmd)
            else:
                # No es la primera detección
                if right_hand:
                    if self.teleop_enabled:
                        # Teleoperación activa: seguir la mano normalmente
                        self._process_hand_to_state(right_hand, right_cmd, is_left=False)
                        self._print_pose("RIGHT", right_cmd)
                        self.right_state_publisher.publish(right_cmd)
                    else:
                        # Teleoperación desactivada: mantener pose inicial, sin control
                        right_cmd.pose.position.x = self.right_initial_position[0]
                        right_cmd.pose.position.y = self.right_initial_position[1]
                        right_cmd.pose.position.z = self.right_initial_position[2]
                        right_cmd.pose.orientation = self.right_initial_orientation
                        right_cmd.twist.linear.x = 0.0
                        right_cmd.twist.linear.y = 0.0
                        right_cmd.twist.linear.z = 0.0
                        right_cmd.twist.angular.x = 0.0
                        right_cmd.twist.angular.y = 0.0
                        right_cmd.twist.angular.z = 0.0
                        right_cmd.enabled = False
                        right_cmd.reset = False
                        self._print_pose("RIGHT", right_cmd)
                        self.right_state_publisher.publish(right_cmd)
        else:
            # No se detecta mano derecha - deshabilitar brazo derecho
            # En lugar de resetear a (0,0,0), mantener la pose inicial del YAML
            self.right_hand_first_detection = True
            self.right_init_counter = 0
            self.right_first_enabled_sent = False  # Reset flag cuando se pierde la mano
            self.right_hand_initial_position = None  # Reset posición inicial de la mano
            self.right_hand_initial_orientation = None  # Reset orientación inicial de la mano
            # Enviar pose inicial del YAML en lugar de (0,0,0)
            right_cmd.pose.position.x = self.right_initial_position[0]
            right_cmd.pose.position.y = self.right_initial_position[1]
            right_cmd.pose.position.z = self.right_initial_position[2]
            right_cmd.pose.orientation = self.right_initial_orientation
            right_cmd.twist.linear.x = 0.0
            right_cmd.twist.linear.y = 0.0
            right_cmd.twist.linear.z = 0.0
            right_cmd.twist.angular.x = 0.0
            right_cmd.twist.angular.y = 0.0
            right_cmd.twist.angular.z = 0.0
            right_cmd.enabled = False  # Deshabilitar control
            right_cmd.reset = False
            self._print_pose("RIGHT", right_cmd)
            self.right_state_publisher.publish(right_cmd)


    def _process_hand_to_state(self, hand: Hand, state_cmd: State, is_left: bool = True):
        """
        Convert Leap Motion Hand data (palm center) to State command.
        Movements are relative to the initial pose from YAML config AND the initial hand position.
        
        Args:
            hand: Leap Motion Hand message containing palm data
            state_cmd: State message to fill with command data
            is_left: True for left arm, False for right arm
        """
        
        # Get initial poses for this arm
        if is_left:
            robot_initial_pos = self.left_initial_position
            robot_initial_orient = self.left_initial_orientation
            hand_initial_pos = self.left_hand_initial_position
            hand_initial_orient = self.left_hand_initial_orientation
        else:
            robot_initial_pos = self.right_initial_position
            robot_initial_orient = self.right_initial_orientation
            hand_initial_pos = self.right_hand_initial_position
            hand_initial_orient = self.right_hand_initial_orientation
        
        # Verificar que tenemos la posición inicial de la mano guardada
        if hand_initial_pos is None or hand_initial_orient is None:
            self.get_logger().warn(f'⚠️ {"LEFT" if is_left else "RIGHT"}: No se guardó posición inicial de la mano, usando valores por defecto')
            # Fallback: usar posición actual de la mano directamente
            hand_delta_x = -hand.position.x * self.position_scale
            hand_delta_y = -hand.position.y * self.position_scale
            hand_delta_z = hand.position.z * self.position_scale
            state_cmd.pose.position.x = robot_initial_pos[0] + hand_delta_x
            state_cmd.pose.position.y = robot_initial_pos[1] + hand_delta_y
            state_cmd.pose.position.z = robot_initial_pos[2] + hand_delta_z
        else:
            # Calcular posición actual de la mano (con escalado e inversiones)
            # IMPORTANTE: Aplicar las mismas transformaciones que cuando se guardó hand_initial_pos
            hand_current_x = -hand.position.x * self.position_scale
            hand_current_y = -hand.position.y * self.position_scale
            hand_current_z = hand.position.z * self.position_scale
            
            # Calcular DELTA: movimiento de la mano desde que se activó teleoperación
            hand_delta_x = hand_current_x - hand_initial_pos[0]
            hand_delta_y = hand_current_y - hand_initial_pos[1]
            hand_delta_z = hand_current_z - hand_initial_pos[2]
            
            # LIMITAR DELTAS para evitar saltos bruscos al inicio
            # Esto previene movimientos inesperados si hay diferencias de frame o ruido
            hand_delta_x = max(-self.max_delta_position, min(self.max_delta_position, hand_delta_x))
            hand_delta_y = max(-self.max_delta_position, min(self.max_delta_position, hand_delta_y))
            hand_delta_z = max(-self.max_delta_position, min(self.max_delta_position, hand_delta_z))
            
            # Aplicar delta a la pose inicial del robot
            # La pose inicial del robot se mantiene como base, solo se aplica el movimiento relativo
            state_cmd.pose.position.x = robot_initial_pos[0] + hand_delta_x
            state_cmd.pose.position.y = robot_initial_pos[1] + hand_delta_y
            state_cmd.pose.position.z = robot_initial_pos[2] + hand_delta_z
        
        # Set palm orientation (con inversiones configurables)
        # Calcular orientación relativa desde que se activó teleoperación
        if hand_initial_orient is None:
            # Fallback: usar orientación actual directamente
            hand_orient = Quaternion()
            if self.invert_roll or self.invert_pitch or self.invert_yaw:
                hand_orient.w = hand.orientation.w
                hand_orient.x = -hand.orientation.x if self.invert_roll else hand.orientation.x
                hand_orient.y = -hand.orientation.y if self.invert_pitch else hand.orientation.y
                hand_orient.z = -hand.orientation.z if self.invert_yaw else hand.orientation.z
            else:
                hand_orient = hand.orientation
            # Combinar con orientación inicial del robot
            state_cmd.pose.orientation = self._multiply_quaternions(robot_initial_orient, hand_orient)
        else:
            # Calcular orientación actual de la mano (con inversiones si aplica)
            hand_current_orient = Quaternion()
            if self.invert_roll or self.invert_pitch or self.invert_yaw:
                hand_current_orient.w = hand.orientation.w
                hand_current_orient.x = -hand.orientation.x if self.invert_roll else hand.orientation.x
                hand_current_orient.y = -hand.orientation.y if self.invert_pitch else hand.orientation.y
                hand_current_orient.z = -hand.orientation.z if self.invert_yaw else hand.orientation.z
            else:
                hand_current_orient = hand.orientation
            
            # Calcular DELTA de orientación: hand_current * hand_initial.inverse()
            # Esto da la rotación relativa desde que se activó teleoperación
            # hand_delta = hand_current * hand_initial^(-1)
            # Representa: "¿Cuánto ha rotado la mano desde que se activó?"
            hand_initial_inv = self._quaternion_inverse(hand_initial_orient)
            hand_delta_orient = self._multiply_quaternions(hand_current_orient, hand_initial_inv)
            
            # Aplicar delta a la orientación inicial del robot
            # robot_final = robot_initial * hand_delta
            # Esto mantiene la orientación inicial del robot como base y aplica solo el cambio relativo
            state_cmd.pose.orientation = self._multiply_quaternions(robot_initial_orient, hand_delta_orient)
        
        # Set palm velocity (scaled) with axis corrections
        # Las velocidades se calculan como el cambio de posición entre frames
        # Para teleoperación posicional, podemos dejarlas en cero o calcularlas del delta
        # Opción 1: Usar velocidades de la mano directamente (comentado por defecto)
        # state_cmd.twist.linear.x = -hand.velocity.x * self.velocity_scale
        # state_cmd.twist.linear.y = -hand.velocity.y * self.velocity_scale
        # state_cmd.twist.linear.z = hand.velocity.z * self.velocity_scale
        
        # Opción 2: Dejar velocidades en cero para control posicional puro
        # Esto hace que el robot se mueva solo a la pose deseada sin seguir velocidades
        state_cmd.twist.linear.x = 0.0
        state_cmd.twist.linear.y = 0.0
        state_cmd.twist.linear.z = 0.0
        
        # Enable control basado en flag global de teleoperación
        # Ya NO dependemos de mantener el puño cerrado.
        # Solo se aplica el comando si teleop_enabled = True.
        state_cmd.enabled = bool(self.teleop_enabled)
        
        # Reset flag (not used in basic teleoperation)
        state_cmd.reset = False
        
        # Log control state periodically (only when state changes)
        # This helps debugging without flooding the console
        # Uncomment if you want to see control state:
        # if state_cmd.enabled:
        #     self.get_logger().info(f'{hand.type} hand control ENABLED (grab: {hand.grab_strength:.2f})', 
        #                           throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = UR5DualPalmTeleoperationPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

