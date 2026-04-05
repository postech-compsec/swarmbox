import plotly.express as px
import plotly.graph_objects as go
import plotly.figure_factory as ff
from plotly.subplots import make_subplots
import plotly.io as pio
import pandas as pd
import numpy as np
from ast import literal_eval
import os

rq4_dir = os.path.dirname(os.path.dirname(__file__))
data_dir = rq4_dir + '/data/'
fig_dir = rq4_dir + '/figures/'
df_util = pd.read_csv(data_dir+'utilization_rate.csv')
df_wkds = pd.read_csv(data_dir+'workload_distance.csv')
df_wkti = pd.read_csv(data_dir+'workload_time.csv')
df_idle = pd.read_csv(data_dir+'idle_time.csv')
df_tmax = pd.read_csv(data_dir+'extra.csv')[['sort','assign','config','delivery_time_max']]
df_tavg = pd.read_csv(data_dir+'extra.csv')[['sort','assign','config','delivery_time_mean']]
df_total_dist = pd.read_csv(data_dir+'extra.csv')[['sort','assign','config','flight_distance']]
df_total_time = pd.read_csv(data_dir+'extra.csv')[['sort','assign','config','mission_duration']]
df_dtime = pd.read_csv(data_dir+'delivery_time.csv')
# df_util.head()

df_util_stats = pd.DataFrame(columns=['sort', 'assign', 'avg', 'std', 'min', 'max'])
df_wkds_stats = pd.DataFrame(columns=['sort', 'assign', 'avg', 'std', 'min', 'max'])
df_wkti_stats = pd.DataFrame(columns=['sort', 'assign', 'avg', 'std', 'min', 'max'])
df_idle_stats = pd.DataFrame(columns=['sort', 'assign', 'avg', 'std', 'min', 'max'])
df_tmax_stats = pd.DataFrame(columns=['sort', 'assign', 'avg', 'std', 'min', 'max'])
df_tavg_stats = pd.DataFrame(columns=['sort', 'assign', 'avg', 'std', 'min', 'max'])
df_total_dist_stats = pd.DataFrame(columns=['sort', 'assign', 'avg', 'std', 'min', 'max'])
df_total_time_stats = pd.DataFrame(columns=['sort', 'assign', 'avg', 'std', 'min', 'max'])
df_dtime_stats = pd.DataFrame(columns=['sort', 'assign', 'delivery_time'])
# columns: ['sort', 'assign', 'avg', 'std', 'min', 'max']
sort_vals = ['No sort', 'Azimuthal', 'Distance']
assi_vals = ['Static-RR', 'Static-Block', 'Dynamic']
# df_util_stats.columns = ['sort', 'assign', 'avg', 'std', 'min', 'max']
# calculate statistics for each sort and assign
dframes = [df_util, df_wkds, df_wkti, df_idle, df_tmax, df_tavg, df_total_dist, df_total_time, df_dtime]
dframe_stats = [df_util_stats, df_wkds_stats, df_wkti_stats, df_idle_stats, df_tmax_stats, df_tavg_stats, df_total_dist_stats, df_total_time_stats, df_dtime_stats]

for sort in range(len(sort_vals)):
    for assign in range(len(assi_vals)):
        for i in range(len(dframes)):
            dframe = dframes[i]
            dstat_frame = dframe_stats[i]
            subset = dframe[(dframe['sort'] == sort) & (dframe['assign'] == assign)]
            # treat them as 1d: 
            # get values of subset[['drone_0', 'drone_1', 'drone_2', 'drone_3', 'drone_4']] and make them as array
            if i < 8:
                if i < 4:
                    values = subset[['drone_0', 'drone_1', 'drone_2', 'drone_3', 'drone_4']].values.flatten()
                elif i == 4:
                    values = subset['delivery_time_max'].values.flatten()
                elif i == 5:
                    values = subset['delivery_time_mean'].values.flatten()
                elif i == 6:
                    values = (subset['flight_distance']/5).values.flatten()
                elif i == 7:
                    values = subset['mission_duration'].values.flatten()
                avg = values.mean()
                std = values.std()
                min_val = values.min()
                max_val = values.max()
                statdata = {
                    'sort': sort,
                    'assign': assign,
                    'avg': avg,
                    'std': std,
                    'min': min_val,
                    'max': max_val
                }
                dstat_frame.loc[len(df_util_stats)] = statdata
            else:
                # for df_dtime, we need to calculate the delivery time for each package
                # so we need to get the values of 'delivery_time' column
                values = []
                for data in subset['delivery_time'].values:
                    values += literal_eval(data)
                # values = subset['delivery_time'].values.flatten()
                statdata = {
                    'sort': sort,
                    'assign': assign,
                    'delivery_time': values
                }
                dstat_frame.loc[len(df_dtime_stats)] = statdata


# draw heatmap, x: sort, y: assign, z: avg
figs = []
details = {
    0: ['Utilization', '%', 'avg', 'util_rate'],
    1: ['Workload StdDev: Distance', 'm', 'std', 'std_dist'],
    2: ['Workload StdDev: Time', 's', 'std', 'std_time'],
    3: ['Idle Time', 's', 'avg', 'idle_time'],
    4: ['Max Delivery Wait', 's', 'avg', 'max_wait'],
    5: ['Avg Delivery Wait', 's', 'avg', 'avg_wait'],
    6: ['Total Flight Distance', 'm', 'avg', 'total_dist'],
    7: ['Total Mission Duration', 's', 'avg', 'total_time'],
}

for i in range(len(dframe_stats)-1):
    dstat_frame = dframe_stats[i]
    heatmap_data = dstat_frame.pivot_table(index='assign', columns='sort', values=details[i][2])
    heatmap_data = heatmap_data.reindex([2,0,1])
    fig = go.Figure(
        data=go.Heatmap(
            z=heatmap_data.values,
            x=["No sort","Azimuthal","Distance"],
            y=["Dynamic", "Pre:RR", "Pre:Block"],
        xgap=2, ygap=2,
        colorscale='greens' if i in [0] else 'greens_r',
        # coloraxis=None,
        showscale=False,
        colorbar=dict(orientation='h', title_side='top', thickness=15,),
        text=heatmap_data.values,
        texttemplate="%{text:.2f}"+details[i][1], 
        textfont=dict(size=20),
        ),
        # showscale=False,
        # title_font=dict(size=10)
    )
    

    fig.update_xaxes(scaleanchor="y", scaleratio=1, 
                    #  rangemode='tozero',
                     constrain='domain',
                    #  title_text='Sort'
                    )
    fig.update_yaxes(scaleanchor="x", scaleratio=1, 
                    #  rangemode='tozero',
                    #  title_text='Assign', 
                    # autorange='reversed',
                    tickangle=-90)

    fig.update_layout(
        # title='Average Utilization Heatmap', 
        title=dict(text= details[i][0],
                   x=0.5, xanchor='center',
                   font=dict(
                       size=24,
                   )
                   ),
        width=400, height=420,

        # plot_bgcolor='black',
        template='simple_white',
        margin=dict(l=0, r=0, t=40, b=0),
        font_family='Fira Code',
        font_color='black',
        font_size=18,
        # font_weight=500,
                    )

    figs.append(fig)
    # fig.write_image(f'{details[i][-1]}.png', scale=2, width=400, height=420)
    fig.write_image(f'{fig_dir}{details[i][-1]}.pdf', width=400, height=420)


# draw boxplot for delivery time
fig_dtime = go.Figure()
for i in range(len(assi_vals)):
    xvals = []
    yvals = []
    for j in [1,0,2]:
        subset = df_dtime_stats[(df_dtime_stats['sort'] == i) & (df_dtime_stats['assign'] == j)]
        xvals += subset['delivery_time'].explode().values.tolist()
        yvals += [assi_vals[j]] * len(subset['delivery_time'].explode())
    box = go.Box(
            y = xvals,  # y-axis is the sort index
            x = yvals,
            name=f"{sort_vals[i]}",
            marker_color=["#3D9970", "#FF4136", "#004A82"][i],
            # marker_color=f'rgba(0, 0, 0, {i * 0.2 + 0.2})',
            # y=i,
            # boxpoints='all',  # Show all points
            boxmean=True,
            # fillcolor=f"rgba(0,0,0,{i * 0.4})",
            # fillcolor='000',
            # opacity=0.5,
        )
    fig_dtime.add_trace(box)
fig_dtime.update_yaxes(
    rangemode='tozero'
)
# fig_dtime.update_traces(
#     orientation='h',  # horizontal boxplot
# )

fig_dtime.update_legends(
    # title=dict(text='Sorting',
    #            font_size=18),
    orientation='h',
    y=1.02,
    yanchor='top',
    xanchor='right',
    x=1,
    font=dict(
        size=16,
        color='black')
)

fig_dtime.update_layout(
    title=dict(
        text='Delivery Time',
        x=0.5, xanchor='center',
        font=dict(size=24)
    ),
    # xaxis_title='Assign Method',
    yaxis_title='Time (s)',
    boxmode='group',
    margin=dict(l=0, r=0, t=40, b=50),
    width=800,
    height=420,
    template='simple_white',
    font_family='Fira Code',
    font_color='black',
    font_size=16,
    # font_weight=500,
)


# fig_dtime.show()

fig_dtime.write_image(f'{fig_dir}delivery_time_boxplot.pdf', width=800, height=420)
