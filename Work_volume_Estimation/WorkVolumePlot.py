import numpy as np
import plotly.graph_objects as go
import matplotlib.pyplot as plt

filename="Work_volume_Estimation/WorkVolume.txt"

def plot_workvolume(filename="Work_volume_Estimation/WorkVolume.txt", save=True):
    data = np.loadtxt(filename)

    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]
    reachable = data[:, 3]

    mask_r = reachable == 1
    mask_n = reachable == 0

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(
        x[mask_r], y[mask_r], z[mask_r],
        s=3, c='green', alpha=0.6, label="Reachable"
    )

    ax.scatter(
        x[mask_n], y[mask_n], z[mask_n],
        s=2, c='red', alpha=0.15, label="Not reachable"
    )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Workvolume (Fixed Vertical Orientation)")

    ax.set_box_aspect([1, 1, 1])
    ax.legend()

    # ---- Fixed camera view (important for consistency)
    ax.view_init(elev=25, azim=135)

    # ---- White background (publication quality)
    fig.patch.set_facecolor("white")

    plt.tight_layout()

    # ---- Save image
    if save:
        plt.savefig("WorkVolume3D.png", dpi=300, bbox_inches="tight")

    plt.show()


def plot_workvolume_interactive_html(filename="Work_volume_Estimation/WorkVolume.txt"):
    data = np.loadtxt(filename)

    # Original points
    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]
    reachable = data[:, 3]

    # Mirrored points (about YZ plane)
    x_m = -x
    y_m = y
    z_m = z
    reachable_m = reachable

    fig = go.Figure()

    # -------- Reachable (original + mirrored) --------
    mask_r = reachable == 1

    fig.add_trace(go.Scatter3d(
        x=np.concatenate([x[mask_r], x_m[mask_r]]),
        y=np.concatenate([y[mask_r], y_m[mask_r]]),
        z=np.concatenate([z[mask_r], z_m[mask_r]]),
        mode='markers',
        marker=dict(size=3, color='green', opacity=0.6),
        name='Reachable'
    ))

    # -------- Not reachable (original + mirrored) --------
    mask_n = reachable == 0

    fig.add_trace(go.Scatter3d(
        x=np.concatenate([x[mask_n], x_m[mask_n]]),
        y=np.concatenate([y[mask_n], y_m[mask_n]]),
        z=np.concatenate([z[mask_n], z_m[mask_n]]),
        mode='markers',
        marker=dict(size=2, color='red', opacity=0.15),
        name='Not Reachable'
    ))

    fig.update_layout(
        title="Workvolume (Fixed Vertical-Down Orientation)",
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)",
            aspectmode="cube"
        ),
        legend=dict(x=0.02, y=0.98)
    )

    fig.write_html("Work_volume_Estimation/WorkVolume3D.html")



if __name__ == "__main__":
    # plot_workvolume()
    plot_workvolume_interactive_html()
