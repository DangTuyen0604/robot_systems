def __init__(self):
    super().__init__('traffic_sign_node')

    # ================= PARAMETERS =================
    self.declare_parameter('imgsz', 320)
    self.declare_parameter('process_every_n_frames', 2)

    self.imgsz = self.get_parameter('imgsz').value
    self.process_every = self.get_parameter('process_every_n_frames').value
    self.frame_count = 0

    # ================= MODEL PATH =================
    package_dir = get_package_share_directory('perception_pkg')
    sign_path = os.path.join(package_dir, 'yolov8n.onnx')
    light_path = os.path.join(package_dir, 'traffic_light.onnx')

    # ================= CLASS NAMES =================
    self.sign_classes = [
        'cam_re_phai',
        'di_cham',
        'dung_lai',
        're_trai',
        're_phai',
        'toc_do_toi_da_20'
    ]

    self.light_classes = [
        'den_do',
        'den_vang',
        'den_xanh'
    ]

    # ================= CPU ONLY =================
    providers = ['CPUExecutionProvider']

    self.get_logger().info('Loading ONNX models (CPU MODE)...')

    sess_options = ort.SessionOptions()
    sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    sess_options.intra_op_num_threads = 4
    sess_options.inter_op_num_threads = 1

    self.sign_session = ort.InferenceSession(
        sign_path,
        sess_options=sess_options,
        providers=providers
    )

    self.light_session = ort.InferenceSession(
        light_path,
        sess_options=sess_options,
        providers=providers
    )

    self.get_logger().info(f'Sign providers: {self.sign_session.get_providers()}')
    self.get_logger().info(f'Light providers: {self.light_session.get_providers()}')

    self.input_name_sign = self.sign_session.get_inputs()[0].name
    self.input_name_light = self.light_session.get_inputs()[0].name

    # ================= ROS =================
    self.label_pub = self.create_publisher(
        String,
        '/perception/detected_label',
        10
    )

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )

    self.subscription = self.create_subscription(
        CompressedImage,
        '/raw_image/compressed',
        self.listener_callback,
        qos
    )

    self.prev_time = time.time()

    self.get_logger().info('TrafficSignNode CPU READY')
