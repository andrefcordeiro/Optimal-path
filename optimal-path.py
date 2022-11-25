import osmnx as ox
import networkx as nx
import plotly.graph_objects as go
import numpy as np
import tkinter
import tkintermapview


locais = []
start = None


def select_positions():
    '''
    Função utiliza para escolher pontos no mapa a serem utilizados no caminho.

    O primeiro ponto selecionado é a origem e o restante são os pontos que demve ser visitados
    '''

    global root_tk

    def add_marker_event(coords):
        global start

        if start is None:
            start = coords
            new_marker = map_widget.set_marker(
                coords[0], coords[1], text="Start", marker_color_outside='#48a2f7', marker_color_circle='white')
        else:
            locais.append(coords)
            new_marker = map_widget.set_marker(
                coords[0], coords[1], text="Local")

    # create tkinter window
    root_tk = tkinter.Tk()
    root_tk.geometry(f"{800}x{600}")
    root_tk.title("map_view_example.py")

    # create map widget
    map_widget = tkintermapview.TkinterMapView(
        root_tk, width=800, height=600, corner_radius=0)
    map_widget.place(relx=0.5, rely=0.5, anchor=tkinter.CENTER)

    map_widget.set_position(-23.319560313133028, -51.12428291797715)
    map_widget.set_zoom(17)

    map_widget.add_left_click_map_command(add_marker_event)

    root_tk.mainloop()


def plot_path(lat, long, origin_point, destination_points):
    """
    Given a list of latitudes and longitudes, origin 
    and destination point, plots a path on a map

    Parameters
    ----------
    lat, long: list of latitudes and longitudes
    origin_point, destination_point: co-ordinates of origin
    and destination
    Returns
    -------
    Nothing. Only shows the map.
    """
    # adding the lines joining the nodes
    fig = go.Figure(go.Scattermapbox(
        name="Path",
        mode="lines",
        lon=long,
        lat=lat,
        marker={'size': 10},
        line=dict(width=4.5, color='blue')))
    # adding source marker
    fig.add_trace(go.Scattermapbox(
        name="Source",
        mode="markers",
        lon=[origin_point[1]],
        lat=[origin_point[0]],
        marker={'size': 12, 'color': "red"}))

    for i in range(0, len(destination_points)):
        dest = destination_points[i]
        # adding destination marker
        fig.add_trace(go.Scattermapbox(
            name=f'Destination {i}',
            mode="markers",
            lon=[dest[1]],
            lat=[dest[0]],
            marker={'size': 12, 'color': 'green'}))

    # getting center for plots:
    lat_center = np.mean(lat)
    long_center = np.mean(long)
    # defining the layout using mapbox_style
    fig.update_layout(mapbox_style="stamen-terrain",
                      mapbox_center_lat=30, mapbox_center_lon=-80)
    fig.update_layout(margin={"r": 0, "t": 0, "l": 0, "b": 0},
                      mapbox={
                          'center': {'lat': lat_center,
                                     'lon': long_center},
                          'zoom': 13})
    fig.show()


def node_list_to_path(G, node_list):
    """
    Given a list of nodes, return a list of lines that together
    follow the path
    defined by the list of nodes.
    Parameters
    ----------
    G : networkx multidigraph
    route : list
        the route as a list of nodes
    Returns
    -------
    lines : list of lines given as pairs ( (x_start, y_start), 
    (x_stop, y_stop) )
    """
    edge_nodes = list(zip(node_list[:-1], node_list[1:]))
    lines = []
    for u, v in edge_nodes:
        # if there are parallel edges, select the shortest in length
        data = min(G.get_edge_data(u, v).values(),
                   key=lambda x: x['length'])
        # if it has a geometry attribute
        if 'geometry' in data:
            # add them to the list of lines to plot
            xs, ys = data['geometry'].xy
            lines.append(list(zip(xs, ys)))
        else:
            # if it doesn't have a geometry attribute,
            # then the edge is a straight line from node to node
            x1 = G.nodes[u]['x']
            y1 = G.nodes[u]['y']
            x2 = G.nodes[v]['x']
            y2 = G.nodes[v]['y']
            line = [(x1, y1), (x2, y2)]
            lines.append(line)

    return lines


def find_optimal_path():
    '''
    Procura o melhor caminho entre os pontos selecionados e o desenha no mapa
    '''

    G = ox.graph_from_point(
        (start[0], start[1]), dist=5000, dist_type='bbox', network_type='drive')

    route = list()
    s = start
    for local in locais:  # encontra o caminho entre cada par de pontos
        # recebe os nós mais próximos das localizações selecionadas
        origin_node = ox.nearest_nodes(G, s[1], s[0])
        destination_node = ox.nearest_nodes(
            G, local[1], local[0])
        s = local

        # Encontra o caminho ótimo e concatena com o anterior
        route = route[:-1] + nx.astar_path(
            G, origin_node, destination_node, weight='length')

    # getting the list of coordinates from the path
    # (which is a list of nodes)
    lines = node_list_to_path(G, route)
    long = []
    lat = []
    for i in range(len(lines)):
        z = list(lines[i])
        l1 = list(list(zip(*z))[0])
        l2 = list(list(zip(*z))[1])
        for j in range(len(l1)):
            long.append(l1[j])
            lat.append(l2[j])

    plot_path(lat, long, start, locais)


select_positions()
print(f'\n\nPonto de origem selecionado: {start}')
print('\nLocais que devem visitados: \n')
for l in locais:
    print(l)
print()

find_optimal_path()
