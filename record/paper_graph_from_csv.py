#!/usr/bin/env python3
import argparse
import os
import matplotlib.pyplot as plt
import numpy as np
# from datetime import datetime
# from zoneinfo import ZoneInfo

from typing import Union, LiteralString, List
import pandas as pd

FileName = Union[LiteralString, str]
DEBUG: bool = False
VERBOSE: bool = False

class ExtractClusterData:

    def __init__(
                self, 
                 data: Union[pd.DataFrame, FileName], 
                 is_offline: bool = True,
                 perform_smoothen: bool = False
                 ) -> None:
        self.is_offline = is_offline

        # Create Panda Frame
        self.data = self.create_panda_dataframe(data)

        # Start and End indices
        self.start: int = 0
        self.end: int = self.data.size

        # Plot indices
        self.i = 0

        self.define_col_headers()    
    
    def define_col_headers(self) -> None:
        
        # For each column in the data frame
        for c in self.data.columns:

            # Create a class attribute and assign the column
            # NOTE: The .strip() method removes leading and trailing
            # whitespace.
            # NOTE: The .replace(' ', '_') replaces inner spaces with
            # underscores
            self.__setattr__(c.strip().replace(' ', '_'), self.data[c])

        # Prints all the keys
        if VERBOSE: print(self.__dict__.keys())  

    def smoothen() -> None:
        pass
    
    def _get_time_domain_fig(
                            self,
                            i, 
                            start: int, 
                            end: int, 
                            y: pd.DataFrame
                            ) -> plt.Figure:

        fig = plt.figure(i)


        plt.plot(self.timestamp[start:end], y[start:end])
        return fig
    

    def get_time_domain_figs(self) -> List[plt.Figure]:

        # Form static data structures
        # TODO: Can this be more generalizable and
        # not hardcoded?
        cluster_vars: dict[int, List[pd.DataFrame]] = \
            {
             1: [self.x_c_des, self.x_c],
             2: [self.y_c_des, self.y_c],
             3: [self.t_c_des, self.t_c],
             4: [self.p_des, self.p],
             5: [self.q_des, self.q],
             6: [self.B_des, self.B],
             }
        
        # Initialize empty list of 6 length
        figs: list = [None] * len(cluster_vars.keys())
        if DEBUG: print(len(figs))

        for k, list_values in cluster_vars.items():
            if DEBUG: print(k)
            for v in list_values:
                figs[k - 1] = \
                    self._get_time_domain_fig(
                    k,
                    self.start,
                    self.end,
                    v
                    )
        return figs
           

    @staticmethod
    def create_panda_dataframe(data) -> pd.DataFrame:
        
        # Get datatype
        typ = type(data)

        # TODO: Check which type of string
        if typ == str or typ == LiteralString:
            return pd.read_csv(data)
        elif typ == pd.DataFrame:
            return data
        


def graph(csv_path):
    data = np.loadtxt(csv_path, delimiter=',', skiprows=1)
    print(data)
    
    ts = data.T[0]
    x_c_des = data.T[9*3+1]
    y_c_des = data.T[9*3+2]
    t_c_des = data.T[9*3+3]
    p_des = data.T[9*3+7]
    q_des = data.T[9*3+8]
    B_des = data.T[9*3+9]

    x_c = data.T[9*2+1]
    y_c = data.T[9*2+2]
    t_c = data.T[9*2+3]
    p = data.T[9*2+7]
    q = data.T[9*2+8]
    B = data.T[9*2+9]

    length = 600*3
    start = 0
    end = length *4

    os.makedirs("figures", exist_ok=True)
    fig = plt.figure()
    plt.plot(x_c_des[start:end], y_c_des[start:end], label="Cluster Desired")
    plt.plot(x_c[start:end], y_c[start:end], label="Cluster")

    plt.title("x_c vs y_c")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.legend()
    fig.savefig(f"figures/cluster.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts[start:end], x_c_des[start:end], label="desired x_c")
    plt.plot(ts[start:end], x_c[start:end], label="actual x_c")
    plt.title("x_c vs t")
    plt.xlabel("t (sec)")
    plt.ylabel("x_c")
    plt.legend()
    fig.savefig(f"figures/x_c.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts[start:end], y_c_des[start:end], label="desired y_c")
    plt.plot(ts[start:end], y_c[start:end], label="actual y_c")
    plt.title("y_c vs t")
    plt.xlabel("t (sec)")
    plt.ylabel("y_c")
    plt.legend()
    fig.savefig(f"figures/y_c.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts[start:end], t_c_des[start:end], label="desired t_c")
    plt.plot(ts[start:end], t_c[start:end], label="actual t_c")
    plt.title("t_c vs t")
    plt.xlabel("t (sec)")
    plt.ylabel("t_c")
    plt.legend()
    fig.savefig(f"figures/y_c.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts[start:end], p_des[start:end], label="desired p")
    plt.plot(ts[start:end], p[start:end], label="actual p")
    plt.title("p vs t")
    plt.xlabel("t (sec)")
    plt.ylabel("p")
    plt.legend()
    fig.savefig(f"figures/p.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts[start:end], q_des[start:end], label="desired q")
    plt.plot(ts[start:end], q[start:end], label="actual q")
    plt.title("q vs t")
    plt.xlabel("t (sec)")
    plt.ylabel("q")
    plt.legend()
    fig.savefig(f"figures/q.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts[start:end], B_des[start:end], label="desired B")
    plt.plot(ts[start:end], B[start:end], label="actual B")
    plt.title("B vs t")
    plt.xlabel("t (sec)")
    plt.ylabel("B")
    plt.legend()
    fig.savefig(f"figures/B.png")
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="ROS 2 bag reader"
    )

    parser.add_argument(
        "datacsv", 
        help="datafiles"
    )
    args = parser.parse_args()
    
    graph(args.datacsv)

def main2():
    
    grapher: ExtractClusterData = ExtractClusterData(
        "record/TestData_with_Timestamp.csv",
    )

    grapher.end = 7200

    figs: List[plt.Figure] = grapher.get_time_domain_figs()

    for f in range(len(figs)):
        print(f)
        figs[f].show()

if __name__ == "__main__":
    main2()

if __name__ == "__main__":
    main2()