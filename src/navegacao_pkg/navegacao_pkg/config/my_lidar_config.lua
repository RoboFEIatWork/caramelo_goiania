MAP_FRAME = "map"
TRACK_FRAME = "base_link"
ODOM_FRAME = "odom"
PROVIDER_FRAME = "base_laser_link"
BODY_FRAME = "base_link"

-- Configurações do sensor LiDAR
LASER_BUILDER = {
  min_range = 0.15,
  max_range = 10.0,
  missing_data_ray_length = 5.0,
  num_theta_points = 180,   -- 180 pontos de laser
  num_rays = 180,           -- 180 raios para uma varredura de 180 graus
  use_given_scan_direction = true,
}

TRAJECTORY_BUILDER_2D = {
  scan_period = 0.1,
  min_range = 0.15,
  max_range = 10.0,
  missing_data_ray_length = 5.0,
  num_lasers = 1,           -- Número de lasers (1 para LiDAR de 1 camada)
  laser_angles = {-1.5708, 1.5708}, -- Ângulos de -90º a +90º em radianos
}

-- Configuração da otimização do grafo de poses
POSE_GRAPH_OPTIMIZATION = {
  optimization_problem = {
    huber_scale = 5.0,
  },
}

-- Adicionando a chave 'trajectory_builder' que será usada pelo Cartographer
TRAJECTORY_BUILDER = {
  trajectory_builder_2d = TRAJECTORY_BUILDER_2D,
}

-- Retornando a tabela com todas as configurações
return {
  MAP_FRAME = MAP_FRAME,
  TRACK_FRAME = TRACK_FRAME,
  ODOM_FRAME = ODOM_FRAME,
  PROVIDER_FRAME = PROVIDER_FRAME,
  BODY_FRAME = BODY_FRAME,
  LASER_BUILDER = LASER_BUILDER,
  TRAJECTORY_BUILDER = TRAJECTORY_BUILDER,
  POSE_GRAPH_OPTIMIZATION = POSE_GRAPH_OPTIMIZATION,
}
