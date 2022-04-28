# graph_tools

Логика функционала взята с interactive_slam с переделкой под интерфейс коммандной строки

# Обединение карт
- roslaunch graph_tools merge_maps.launch map_1_path:="" map_2_path:="" save_path:="" export_pcd:=1

- в файле "merge_maps.launch" нужно задать
- "map_1_path" - папка с первой картой
- "map_2_path" - папка со второй картой
- "save_path" - обьединенная карта
- "export_pcd" - нужно ли экспортировать облако (0 - false, 1 - true)

# odometry_to_graph
- roslaunch graph_tools odometry_to_graph.launch odom_path:="" save_path:=""

- "odom_path" - папка с одометрией
- "save_path" - папка с результатом
- "delta_x" - пороговое смещение 
- "delta_angle" - пороговый поворот (В ГРАДУСАХ)
- "downsample_resolution" - вокселизация (чем меньше, тем плотнее точки)
- "inf_pos" - information для позиции между фреймами
- "inf_rot" - information для пооворота между фреймами

# Caution!
- Перед записью папка 'save_path' будет очищена.
- Если папка 'save_path' отсутствует - будет создана автоматически
- Первый фрейм не записывается как "Fixed", в отличии от interactive_slam