import pandas as pd
import plotly.express as px
import plotly.graph_objects as go

from plotly.subplots import make_subplots
fig1 = make_subplots(
    rows=2, cols=3,
    subplot_titles=("Error_x", "Error_y", "Error_z", "Velocity_error_x" ,"Velocity_error_y","Velocity_error_z"))


df = pd.read_csv('/media/swati/MY PASSPORT/rosbags/test2.csv')


fig1.add_trace(go.Scatter(x=df['%time'], y=df['field.position_error.x']), row=1, col=1)
fig1.add_trace(go.Scatter(x=df['%time'], y=df['field.position_error.y']), row=1, col=2)
fig1.add_trace(go.Scatter(x=df['%time'], y=df['field.position_error.z']), row=1, col=3)
fig1.add_trace(go.Scatter(x=df['%time'], y=df['field.velocity_error.x']), row=2, col=1)
fig1.add_trace(go.Scatter(x=df['%time'], y=df['field.velocity_error.y']), row=2, col=2)
fig1.add_trace(go.Scatter(x=df['%time'], y=df['field.velocity_error.z']), row=2, col=3)


fig1.show()

fig2 = px.line(df, x='%time', y=['field.curpos.x', 'field.despos.x'])
fig2.show()