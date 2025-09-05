#!/usr/bin/env python3
import argparse
import os
import shutil
import matplotlib.pyplot as plt
import numpy as np
import yaml

from typing import Union, LiteralString, List, Set, Tuple
import pandas as pd

import matplotlib.tri as tri
from mpl_toolkits.mplot3d import Axes3D  # enables 3D plotting

FileName = Union[LiteralString, str]
Numeric = Union[int, float]
DEBUG: bool = False
VERBOSE: bool = False



class ExtractClusterData:

    def __init__(
                self, 
                 data: Union[pd.DataFrame, FileName], 
                 is_offline: bool = True,
                 number_cluster_vars: int = 6,
                 perform_smoothen: bool = False
                 ) -> None:
        
        # Attributes
        self.is_offline: bool = is_offline
        self.number_cluster_vars: int = number_cluster_vars
        self.perform_smoothen: bool = perform_smoothen

        # Create Panda Frame
        self.data = self.create_panda_dataframe(data)

        # Start and End indices
        self.start: int = 0
        self.end: int = self.data.size

        # Figs
        self.figs: Dict[int, plt.Fig] = dict()

        self.define_col_headers()    
    
    def define_col_headers(self) -> None:
        
        # For each column in the data frame
        for c in self.data.columns:

            # Create a class attribute and assign the column
            # NOTE: The .strip() method removes leading and trailing
            # whitespace.
            # NOTE: The .replace(' ', '_') replaces inner spaces with
            # underscores
            self.__setattr__(c.strip()\
                            .replace(' ', '_')\
                            .replace('/', '_')\
                            .replace('.', '')
                            , self.data[c]
                            )

        # Prints all the keys
        if VERBOSE: print(self.__dict__.keys())  

    def smoothen() -> None:
        pass

    @staticmethod
    # NOTE: From `https://stackoverflow.com/questions/34017866/arrow-on-a-line-plot`
    def add_arrow(line, position=None, direction='right', size=15, color=None):
        """
        add an arrow to a line.

        line:       Line2D object
        position:   x-position of the arrow. If None, mean of xdata is taken
        direction:  'left' or 'right'
        size:       size of the arrow in fontsize points
        color:      if None, line color is taken.
        """
        if color is None:
            color = line.get_color()

        xdata = line.get_xdata()
        ydata = line.get_ydata()

        if position is None:
            position = xdata.mean()
        # find closest index
        start_ind = np.argmin(np.absolute(xdata - position))
        if direction == 'right':
            end_ind = start_ind + 1
        else:
            end_ind = start_ind - 1

        line.axes.annotate('',
            xytext=(xdata[start_ind], ydata[start_ind]),
            xy=(xdata[end_ind], ydata[end_ind]),
            arrowprops=dict(arrowstyle="->", color=color),
            size=size
        )

    # NOTE: Got from ChatGPT
    @staticmethod
    def add_arrows_to_line(line, n_arrows=20, arrow_style='->', color=None, size=15, ind_spacing=10):
        """
        Adds arrows to a matplotlib Line2D object to indicate direction.

        Parameters:
        - line        : matplotlib Line2D object (e.g. from ax.plot)
        - n_arrows    : number of arrows to draw
        - arrow_style : arrow style (e.g., '->', '-|>')
        - color       : arrow color (defaults to line color)
        - size        : arrow size
        - spacing     : 'data' or 'index' based spacing
        """
        import numpy as np
        import matplotlib.pyplot as plt

        x = np.array(line.get_xdata())
        y = np.array(line.get_ydata())
        ax = line.axes  # get the Axes the line belongs to

        if color is None:
            color = line.get_color()

        # if spacing == 'data':
        #     dist = np.cumsum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
        #     dist = np.insert(dist, 0, 0)
        #     arrow_locs = np.linspace(0, dist[-1], n_arrows + 2)[1:-1]
        #     arrow_indices = [np.searchsorted(dist, loc) for loc in arrow_locs]
        # else:
        arrow_indices = np.linspace(1, len(x)-2, n_arrows).astype(int)


        s = ind_spacing//2
        for i in arrow_indices:
            
            next = i + s if i + s < len(x) else len(x) - 1
            prev = i - s if i - s > 0 else 0

            dx = x[next] - x[prev]
            dy = y[next] - y[prev]
            ax.annotate('', xy=(x[next], y[next]), xytext=(x[prev], y[prev]),
                        arrowprops=dict(arrowstyle=arrow_style, color=color),
                        size=size)

    def _get_fig(self,
                i, 
                start: int, 
                end: int, 
                x: pd.DataFrame,
                y: pd.DataFrame,
                use_arrows: bool = False,
                subplot_tuple: Tuple[int] = (1, 1, 1),
                figsize: Tuple[Numeric] = None,
                *args,
                **kwargs
                ) -> plt.Figure:
        
        
        # Create fig
        fig = plt.figure(num=i, figsize=figsize)

        # TODO: Make this configurable? Adjust spacing 
        plt.subplots_adjust(hspace=0.5)
        
        # Extract vars
        row, col, ax_ind = subplot_tuple

        # Check if the axis already exists in the figure
        if len(fig.axes) >= ax_ind:  # If axes exist at ax_ind, use it
            ax = fig.axes[ax_ind - 1] 
            _ax_ind: int = ax_ind - 1
        else: 
            # Otherwise, create a new subplot
            ax = fig.add_subplot(row, col, ax_ind)
            _ax_ind: int = -1

        # Create line
        line, = fig.axes[_ax_ind].plot(x[start:end], y[start:end], *args, **kwargs)

        # Add arrows if desired
        if use_arrows: self.add_arrows_to_line(line)
        
        # Add figure to figs
        self.figs[i] = fig
        

        return fig

    
    def _get_time_domain_fig(
                            self,
                            i, 
                            start: int, 
                            end: int, 
                            y: pd.DataFrame,
                            subplot_tuple: Tuple[int] = None,
                            figsize: Tuple[Numeric] = None,
                            *args,
                            **kwargs
                            ) -> plt.Figure:

        return self._get_fig(i, 
                            start, 
                            end, 
                            self.timestamp, 
                            y, 
                            subplot_tuple = subplot_tuple, 
                            figsize= figsize,
                            *args, **kwargs)
    

    def get_time_domain_figs(
                        self, 
                        ylims: List[Tuple[int]] = None, 
                        use_multi_y_axis: bool = False,
                        use_subplots: bool = False,
                        figsize: Tuple[int] = (6, 14)
                        ) -> List[plt.Figure]:

        # Form static data structures
        # TODO: Can this be more generalizable and
        # not hardcoded?
        cluster_vars: Dict[int, List[pd.DataFrame]] = \
            {
             1: [self.x_c_des, self.x_c],
             2: [self.y_c_des, self.y_c],
             3: [self.t_c_des, self.t_c],
             4: [self.p_des, self.p],
             5: [self.q_des, self.q],
             6: [self.B_des, self.B],
             }
        
        # Form static data structures
        # TODO: Can this be more generalizable and
        # not hardcoded?
        theta_rstr: str = '\theta'
        title_vars: Dict[int, List[str]] = \
            {
                1: ["X-position\n of centroid $x_c$",
                   "X-Position (m)"],
                2: ["Y-position\n of centroid $y_c$",
                   "Y-Position (m)"],
                3: ["Heading\n of centroid $\\theta_c$",
                   f"Heading (rad)"],
                4: ["$p$-length\n of cluster configuration",
                   "Length (m)"],
                5: ["$q$-length\n of cluster configuration",
                   "Length (m)"],
                6: ["$\\beta$-angle\n of cluster configuration",
                   "Angle (rad)"]
            }
        
        # Form static data structures
        # TODO: Can this be more generalizable and
        # not hardcoded?
        if ylims == None:
            ylims = [(-15, 15), (-15, 15), (-4, 4), (0, 15), (0, 15), (-4, 4)]

        # Initialize empty list of 6 length
        if use_subplots:
            figs: list = [None]
        else:
            figs: list = [None] * len(cluster_vars.keys())
        if DEBUG: print(len(figs))

        for k, list_values in cluster_vars.items():
            if DEBUG: print(k)

            # TODO: Fix hardcorded tuples
            if use_subplots:
                subplot_tuple: Tuple[int] = (int(len(cluster_vars.keys())/2), 2, k)
            else:
                subplot_tuple: Tuple[int] = (1, 1, 1)
            for v in list_values:

                # If using subplots
                ind: int = 0 if use_subplots else k-1

                figs[ind] = \
                    self._get_time_domain_fig(
                    ind,
                    self.start,
                    self.end,
                    v,
                    subplot_tuple,
                    figsize=figsize
                    )
                plt.title(title_vars[k][0], fontsize=16, fontweight='bold')
                plt.xlabel("Time (s)")
                plt.ylabel(title_vars[k][1])
                plt.legend(["Desired", "Actual"])
                plt.ylim(ylims[k-1])
                plt.grid(True)

                if use_subplots:
                    # Tight layout
                    plt.tight_layout() 
        return figs
           
    def get_centroid_phase_portrait(
                           self,
                           ind: int,
                           start: int,
                           end: int
                           ) -> plt.Figure:

        fig = self._get_fig(ind, start, end, self.x_c_des, self.y_c_des, use_arrows= True, color="blue")
        fig = self._get_fig(ind, start, end, self.x_c, self.y_c, use_arrows= True, color="green")
        plt.title("Phase portrait of cluster centroid")
        plt.xlabel("X-position of cluster centroid $x_c$ (m)")
        plt.ylabel("Y-position of cluster centroid $y_c$ (m)")
        plt.legend(["Desired", "Actual"])
        plt.grid(True)
        plt.axis('equal')

        return fig

    @staticmethod
    def annotate_polygon(
                        ind: int,
                        x: List[Numeric],
                        y: List[Numeric],
                        label: str = None,
                        include_label: bool = True,
                        *args,
                        **kwargs 
                        ) -> plt.Figure:
        
        # If x and y do not match length
        if len(x) != len(y):
            raise("Incorrect size")

        fig = plt.figure(ind)

        # Plot triangle
        plt.plot(x, y, *args, **kwargs)

        colors: List[str] = ["red", "blue", "green", "orange", "purple"]
        # Add vertices
        for i in range(len(x)):

            plt.scatter(x[i], y[i], color=colors[i % len(x) - 1])

        # Add label
        # Default to labeling the first vertex as "start"
        if include_label: 

            if label:
                p = 0
                start_point = [x[p], y[p]]
                ax = fig.gca()
                ax.text(start_point[0] - 5, 
                        start_point[1] - 0.1, 
                        label, 
                        fontsize=12, 
                        color='Black')


        plt.axis('equal')
        
        return fig
        

    def get_cluster_phase_portrait(
                          self,
                           ind: int,
                           start: int,
                           end: int,
                           include_label: bool = True,
                           ) -> plt.Figure:
        
        # Form static data structures
        # TODO: Can this be more generalizable and
        # not hardcoded?
        xy_pos: Dict[int, List[pd.DataFrame]] = \
            {1: [self._p1_pose2D_x, self._p1_pose2D_y],
            2: [self._p2_pose2D_x, self._p2_pose2D_y],
            3: [self._p3_pose2D_x, self._p3_pose2D_y]}
            # {1: [self.x_1, self.y_1],
            #  2: [self.x_2, self.y_2],
            #  3: [self.x_3, self.y_3]}
        
        for _, v in xy_pos.items():
            fig = \
                self._get_fig(
                ind,
                start,
                end,
                v[0],
                v[1],
                use_arrows= True
                )

        # plt.title("Phase portrait of 3-cluster configuration of robots")
        plt.xlabel("X-position $x$ (m)")
        plt.ylabel("Y-position $y$ (m)")



        # TODO: Can these endpoints be implemented within the annotate_polygon() 
        # instead
        # Creating the points of the cluster config
        x_endpoints: List[Numeric] = [v[0][end] for _, v in xy_pos.items()]
        y_endpoints: List[Numeric] = [v[1][end] for _, v in xy_pos.items()]

        # Adding the first point again to the end to close the polygon
        x_endpoints.append(x_endpoints[0])
        y_endpoints.append(y_endpoints[0])

        if DEBUG:
            print(x_endpoints)
            print(y_endpoints)

        fig = self.annotate_polygon(ind, x_endpoints, y_endpoints, label="End", include_label= include_label, color="blue")

        # Creating the points of the cluster config
        x_startpoints: List[Numeric] = [v[0][start] for _, v in xy_pos.items()]
        y_startpoints: List[Numeric] = [v[1][start] for _, v in xy_pos.items()]

        # Adding the first point again to the end to close the polygon
        x_startpoints.append(x_startpoints[0])
        y_startpoints.append(y_startpoints[0])

        fig = self.annotate_polygon(ind, x_startpoints, y_startpoints, label="Start", include_label= include_label, color="blue")
    
        # TODO: Fix hardcoded number of robots
        plt.legend([f"Robot {i}, $r_{i}$" for i in range(1,4)])
        plt.grid(True)

        return fig

    def plot_contour_map(self) -> plt.Figure:
        
        # TODO: Fix hardcoded values
        _Z = self._p1_rssi_data
        _X = self._contour_x
        _Y = self._contour_y

        n1 = 100
        n2 = 200
        df = pd.DataFrame({
            "/contour._x": _X[n1:n2],
            "/contour._y": _Y[n1:n2],
            "/p1/rssi._data": _Z[n1:n2],
        })


        df["/contour._x"] = df["/contour._x"].round(2)
        df["/contour._y"] = df["/contour._y"].round(2)

        df = df.drop_duplicates(
            subset=["/contour._x", "/contour._y"]
        )

        Z = df.pivot_table(index='/contour._x',
                 columns='/contour._y',
                  values='/p1/rssi._data',
                  aggfunc='mean',
                  fill_value=0.0
                  ).T


        X_unique = np.sort(df["/contour._x"].unique())
        Y_unique = np.sort(df["/contour._y"].unique())

        X, Y = np.meshgrid(X_unique, Y_unique)
        
        print(X_unique)
        print(pd.DataFrame(Z).round(3))

        fig = plt.figure()
        ax = fig.add_subplot(111)

        # Generate a contour plot
        cp = ax.contour(X, Y, Z)

        return pd.DataFrame(Z).round(3) #df

    # TODO: Add histogram plot
    def get_sensor_histogram_plot() -> plt.Figure:
        pass


    def export_fig(
                self, 
                ind: int, 
                filename: FileName = None, 
                dirname: FileName = "figures",  
                ext: str= ".png",
                dir_exists_ok: bool = True,
                ) -> bool:

        try:
                            
            # Create directory
            os.makedirs(dirname, exist_ok=dir_exists_ok)

            # Get figure
            fig = self.figs[ind]

            if filename == None:

                # Get current axis
                ax = fig.axes[0]

                # Extract title and replace name
                filename: str = ax.get_title() \
                                        .lower() \
                                        .replace(' ', '_') \
                                        .replace('\\n', '') \
                                        .replace('$', '') \
                                        .replace('\\','') \
                                        .replace('/', '-')
                if VERBOSE: print(f"Filename: {file_name}")
            
            fig.savefig(os.path.join(dirname, filename + ext))
            if VERBOSE: print("Saved figure!")

        except Exception as e:
            print(f"Error: {e}")
            return False
        return True
        

    def export_figs(self, dirname: FileName = "figures", ext: str= ".png") -> bool:

        try:
            
            # Create directory
            os.makedirs(dirname, exist_ok=True)

            for k, fig in self.figs.items():

                # Get current axis
                ax = fig.axes[0]

                # Extract title and replace name
                file_name: str = ax.get_title() \
                                        .lower() \
                                        .replace(' ', '_') \
                                        .replace('\n', '') \
                                        .replace('$', '') \
                                        .replace('\\','') \
                                        .replace('/', '-')
                if VERBOSE: print(f"Filename: {file_name}")
                
                fig.savefig(os.path.join(dirname,file_name + ext))
                if VERBOSE: print("Saved figure!")

        except Exception as e:
            print(f"Error: {e}")
            return False
        return True
        
    def clear_fig(self, ind: int = -1) -> None:
        self.figs[ind].clf()
        
    def clear_figs(self) -> None:
        for _, f in self.figs.items():
            self.clear_fig(f.number)


    @staticmethod
    def create_panda_dataframe(data) -> pd.DataFrame:
        
        # Get datatype
        typ = type(data)

        # TODO: Check which type of string
        if typ == str or typ == LiteralString:
            return pd.read_csv(data)
        elif typ == pd.DataFrame:
            return data           


contour_data = ExtractClusterData(data='output_09_05_25_191106.csv')

z= contour_data._p1_rssi_data
x= contour_data._contour_x
y = contour_data._contour_y

# plt.tricontour(x, y, z, 15, linewidths=0.5, colors='k')
# plt.tricontourf(x, y, z, 15)
# plt.show()

# Create triangulation for irregular data
triang = tri.Triangulation(x, y)

# Plot contour lines and filled contours
fig, ax = plt.subplots()
contour_lines = ax.tricontour(triang, z, levels=15, linewidths=0.05, colors='k')
contour_filled = ax.tricontourf(triang, z, levels=15, cmap='coolwarm')

# Add colorbar
fig.colorbar(contour_filled, ax=ax)

# Optional: scatter original points
# ax.scatter(x, y, c=z, edgecolor='k', s=0.05)

# Enable interactive navigation toolbar
ax.set_title("Interactive Tricontour Plot")
plt.show()


# Create figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Triangulate the irregular data
triang = tri.Triangulation(x, y)

# Create a 3D surface plot
surf = ax.plot_trisurf(triang, z, cmap='viridis', edgecolor='none')

# Optional: Add contour lines on the surface
# ax.plot_trisurf(triang, z, cmap='viridis', edgecolor='k', alpha=0.2)

# Add colorbar
fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)

# Labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('RSSI')
ax.set_title('3D Surface of RSSI Data')

# Show interactive plot
plt.show()



