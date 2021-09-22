from hy_src.env_base import env_base
from hy_src.hybrid_astar import hybrid_astar
from hy_src.grid_graph import grid_graph
from pathlib import Path
import numpy as np
import cProfile


def main(config_file_path,reeds_path,map_path,show_map,draw_cartoon):
    world_path = config_file_path
    reeds_lookup_path = reeds_path
    world_map = map_path

    env0 = env_base(world_path, world_map)
    env0.initialization()

    grid = grid_graph(env0.map_matrix, xy_reso=env0.xy_reso, yaw_reso=env0.yaw_reso)

    hy_astar = hybrid_astar(grid, env0.car.shape, step_size=2, reeds_size=1, min_radius=9, test_plot=env0,
                            lookup_path=reeds_lookup_path)

    # hy_astar.reeds_lookup_cal()
    # cProfile.run('hy_astar.hy_astar_search(env0.car.state, env0.car.goal, show_process=True)', sort=1)

    if show_map:
        env0.world.com_cla()
        arr=[]
        for i in env0.acker_init_state_list[0]:
            arr.append([i])
        env0.car.update_state(np.array(arr))
        # env0.world.path_plot(path_list, path_color='r', show_point=False)
        env0.render(0.1)
        env0.show()

    path_list = hy_astar.hy_astar_search(env0.car.state, env0.car.goal, show_process=True)

    if draw_cartoon:
        env0.world.path_plot(path_list, path_color='r', show_point=False)
        env0.render(0.1)

        for point in path_list:
            # print(point,type(point))
            env0.world.com_cla()
            env0.car.update_state(point)
            env0.world.path_plot(path_list, path_color='r', show_point=False)

            env0.render(0.1)

        env0.show()
    return path_list

if __name__ == '__main__':

    cur_path = Path(__file__).parent
    world_path = str(cur_path / 'hy_astar_world.yaml')
    reeds_lookup_path = str(cur_path / 'reeds_lookup.npy')
    world_map = str(cur_path / 'map_image' / 'map_100_100.png')

    path=main(world_path,reeds_lookup_path,world_map,show_map=False,draw_cartoon=False)
    for point in path:
        print(point)