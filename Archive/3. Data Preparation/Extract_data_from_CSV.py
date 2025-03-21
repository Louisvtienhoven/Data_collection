import csv
import numpy as np
import os
import panel as pn
import plotly.graph_objects as go
from scipy.fft import fft, fftfreq
from scipy.signal import spectrogram

pn.extension('plotly')


def read_sensor_data(file_path):
    data = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            if len(row) == 6:
                try:
                    float_row = list(map(float, row))
                    data.append(float_row)
                except ValueError:
                    continue
            elif len(row) == 7:
                try:
                    float_row = list(map(float, row[1:]))
                    data.append(float_row)
                except ValueError:
                    continue
    if data:
        return np.array(data).T
    else:
        return np.array([])


def generate_time_list(sampling_frequency, num_samples, start_time=0.0):
    dt = 1.0 / sampling_frequency
    return [start_time + i * dt for i in range(num_samples)]


colors = {
    'neutral space': '#e8e8e4',
    'deep space': '#003247'
}


def ajuster_liste_puissance_de_2(lst):
    n = len(lst)
    if n != 0 and (n & (n - 1)) == 0:
        return np.array(lst)
    next_power = 2 ** ((n - 1).bit_length())
    return np.pad(lst, (0, next_power - n), 'constant')


def corr(signal_ampl, fs_local):
    signal_ampl = np.array(signal_ampl)
    mean_signal = np.mean(signal_ampl)
    signal_centered = signal_ampl - mean_signal
    autocorr_values = np.correlate(signal_centered, signal_centered, mode='full')
    lags = np.arange(-len(signal_ampl) + 1, len(signal_ampl))
    return lags / fs_local, autocorr_values


def psd(signal_ampl, fs_local):
    signal_ampl = np.array(signal_ampl)
    signal_padded = ajuster_liste_puissance_de_2(signal_ampl)
    N = len(signal_padded)
    fft_vals = np.fft.fft(signal_padded)
    freqs = np.fft.fftfreq(N, d=1 / fs_local)
    psd_values = np.abs(fft_vals) ** 2 / N
    positive_mask = freqs > 0
    return freqs[positive_mask], psd_values[positive_mask]


def FFT_signal(signal_ampl, fs_local, limited_harmonique_number=100000):
    signal_ampl = np.array(signal_ampl)
    N = len(signal_ampl)
    yf = np.fft.fft(signal_ampl)
    xf = np.fft.fftfreq(N, d=1 / fs_local)
    positive_mask = xf > 0
    return xf[positive_mask], np.abs(yf[positive_mask])


def FFT_signal_scipy(signal_ampl, fs_local, limited_harmonique_number=100000):
    N = len(signal_ampl)
    yf = fft(signal_ampl)
    xf = fftfreq(N, d=1 / fs_local)
    positive_mask = xf > 0
    return xf[positive_mask], np.abs(yf[positive_mask])


def plot_save_harmonique(signal_data, time, limited_harmonique_number=100000):
    dt = time[1] - time[0]
    fs_local = 1 / dt
    signal_to_plot = signal_data.copy()
    fig_temp = go.Figure()
    fig_temp.add_trace(go.Scatter(x=time, y=signal_to_plot, mode="lines", name="Signal"))
    fig_temp.update_layout(
        title="Signal Evolution",
        xaxis_title="Time (sec)",
        yaxis_title="Amplitude",
        paper_bgcolor=colors['neutral space'],
        plot_bgcolor=colors['neutral space']
    )
    freq, ampl = FFT_signal(signal_to_plot, fs_local, limited_harmonique_number)
    fig_fft = go.Figure()
    fig_fft.add_trace(go.Scatter(x=freq, y=ampl, mode="lines", name="Signal FFT"))
    fig_fft.update_layout(
        title="Fourier Transform (FFT)",
        xaxis_title="Frequency (Hz)",
        yaxis_title="Amplitude",
        paper_bgcolor=colors['neutral space'],
        plot_bgcolor=colors['neutral space']
    )
    lag, corr_val = corr(signal_to_plot, fs_local)
    fig_corr = go.Figure()
    fig_corr.add_trace(go.Scatter(x=lag, y=corr_val, mode="lines", name="Correlation"))
    fig_corr.update_layout(
        title="Correlation Function",
        xaxis_title="Time (sec)",
        yaxis_title="Amplitude",
        paper_bgcolor=colors['neutral space'],
        plot_bgcolor=colors['neutral space']
    )
    freq_psd, psd_val = psd(signal_to_plot, fs_local)
    fig_psd = go.Figure()
    fig_psd.add_trace(go.Scatter(x=freq_psd, y=psd_val, mode="lines", name="Signal PSD"))
    fig_psd.update_layout(
        title="Power Spectral Density (PSD)",
        xaxis_title="Frequency (Hz)",
        yaxis_title="Amplitude",
        paper_bgcolor=colors['neutral space'],
        plot_bgcolor=colors['neutral space']
    )
    return fig_temp, fig_fft, fig_corr, fig_psd


def plot_superpose_acc_yz(time, signal_x, signal_y, signal_z):
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=time, y=signal_x, mode="lines", name="Acceleration X"))
    fig.add_trace(go.Scatter(x=time, y=signal_y, mode="lines", name="Acceleration Y"))
    fig.add_trace(go.Scatter(x=time, y=signal_z, mode="lines", name="Acceleration Z"))
    fig.update_layout(
        title="Superposition of X, Y, and Z Accelerations",
        xaxis_title="Time (sec)",
        yaxis_title="Amplitude",
        paper_bgcolor=colors['neutral space'],
        plot_bgcolor=colors['neutral space']
    )
    return fig


def plot_superpose_gyro(time, signal_x, signal_y, signal_z):
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=time, y=signal_x, mode="lines", name="Gyroscope X"))
    fig.add_trace(go.Scatter(x=time, y=signal_y, mode="lines", name="Gyroscope Y"))
    fig.add_trace(go.Scatter(x=time, y=signal_z, mode="lines", name="Gyroscope Z"))
    fig.update_layout(
        title="Superposition of Gyroscope Signals",
        xaxis_title="Time (sec)",
        yaxis_title="Angular Velocity",
        paper_bgcolor=colors['neutral space'],
        plot_bgcolor=colors['neutral space']
    )
    return fig


def plot_superpose_fft(list_signals, fs_local, limited_harmonique_number=100000, labels=None):
    fig = go.Figure()
    if labels is None:
        labels = ["FFT X", "FFT Y", "FFT Z"]
    for i, sig in enumerate(list_signals):
        freq, ampl = FFT_signal(sig, fs_local, limited_harmonique_number)
        trace_name = labels[i] if i < len(labels) else f"FFT {i}"
        fig.add_trace(go.Scatter(x=freq, y=ampl, mode="lines", name=trace_name))
    fig.update_layout(
        title="Superposition of FFTs",
        xaxis_title="Frequency (Hz)",
        yaxis_title="Amplitude",
        paper_bgcolor=colors['neutral space'],
        plot_bgcolor=colors['neutral space']
    )
    return fig


def plot_superpose_corr(list_signals, fs_local, labels=None):
    fig = go.Figure()
    if labels is None:
        labels = ["Autocorrelation X", "Autocorrelation Y", "Autocorrelation Z"]
    for i, sig in enumerate(list_signals):
        lag, corr_val = corr(sig, fs_local)
        trace_name = labels[i] if i < len(labels) else f"Autocorrelation {i}"
        fig.add_trace(go.Scatter(x=lag, y=corr_val, mode="lines", name=trace_name))
    fig.update_layout(
        title="Superposition of Autocorrelations",
        xaxis_title="Time (sec)",
        yaxis_title="Amplitude",
        paper_bgcolor=colors['neutral space'],
        plot_bgcolor=colors['neutral space']
    )
    return fig


def plot_spectrogram(signal, fs_local, axis_label=""):
    f, t_spec, Sxx = spectrogram(signal, fs=fs_local)
    Sxx_db = 10 * np.log10(Sxx + 1e-10)
    fig = go.Figure(data=go.Heatmap(x=t_spec, y=f, z=Sxx_db, colorscale='Viridis', colorbar=dict(title="dB")))
    fig.update_layout(
        title=f"Spectrogram {axis_label}",
        xaxis_title="Time (sec)",
        yaxis_title="Frequency (Hz)",
        paper_bgcolor=colors['neutral space'],
        plot_bgcolor=colors['neutral space']
    )
    return fig


def update_dashboard(file_name):
    data_arrays = read_sensor_data(file_name)
    if data_arrays.size == 0:
        return pn.pane.Markdown("**Error: No data found in the selected file.**")
    ax, ay, az, gx, gy, gz = data_arrays
    fs = 26
    n_samples = len(gz)
    temps = generate_time_list(fs, n_samples)

    # Accelerometer plots
    row_acc = pn.Row(pn.pane.Plotly(plot_superpose_acc_yz(temps, ax, ay, az), sizing_mode="stretch_width"))
    row_fft = pn.Row(pn.pane.Plotly(plot_superpose_fft([ax, ay, az], fs), sizing_mode="stretch_width"))
    row_corr = pn.Row(pn.pane.Plotly(plot_superpose_corr([ax, ay, az], fs), sizing_mode="stretch_width"))
    spec_x = pn.Row(pn.pane.Plotly(plot_spectrogram(ax, fs, axis_label="Accelerometer X"), sizing_mode="stretch_width"))
    spec_y = pn.Row(pn.pane.Plotly(plot_spectrogram(ay, fs, axis_label="Accelerometer Y"), sizing_mode="stretch_width"))
    spec_z = pn.Row(pn.pane.Plotly(plot_spectrogram(az, fs, axis_label="Accelerometer Z"), sizing_mode="stretch_width"))
    row_spec = pn.Column(spec_x, spec_y, spec_z, sizing_mode="stretch_width")

    # Gyroscope plots
    row_gyro = pn.Row(pn.pane.Plotly(plot_superpose_gyro(temps, gx, gy, gz), sizing_mode="stretch_width"))
    row_fft_gyro = pn.Row(pn.pane.Plotly(
        plot_superpose_fft([gx, gy, gz], fs, labels=["FFT Gx", "FFT Gy", "FFT Gz"]), sizing_mode="stretch_width"))
    row_corr_gyro = pn.Row(pn.pane.Plotly(
        plot_superpose_corr([gx, gy, gz], fs,
                            labels=["Autocorrelation Gx", "Autocorrelation Gy", "Autocorrelation Gz"]),
        sizing_mode="stretch_width"))
    spec_gx = pn.Row(pn.pane.Plotly(plot_spectrogram(gx, fs, axis_label="Gyroscope X"), sizing_mode="stretch_width"))
    spec_gy = pn.Row(pn.pane.Plotly(plot_spectrogram(gy, fs, axis_label="Gyroscope Y"), sizing_mode="stretch_width"))
    spec_gz = pn.Row(pn.pane.Plotly(plot_spectrogram(gz, fs, axis_label="Gyroscope Z"), sizing_mode="stretch_width"))
    row_spec_gyro = pn.Column(spec_gx, spec_gy, spec_gz, sizing_mode="stretch_width")

    # Order of plots:
    # 1. Time Domain Plots (Accelerometer and Gyroscope)
    # 2. FFT Plots (Accelerometer and Gyroscope)
    # 3. Autocorrelation Plots (Accelerometer and Gyroscope)
    # 4. Spectrograms (Accelerometer and Gyroscope)
    return pn.Column(
        pn.pane.Markdown("## Time Domain Plots"),
        row_acc,
        row_gyro,
        pn.pane.Markdown("## FFT Plots"),
        row_fft,
        row_fft_gyro,
        pn.pane.Markdown("## Autocorrelation Plots"),
        row_corr,
        row_corr_gyro,
        pn.pane.Markdown("## Spectrograms"),
        row_spec,
        row_spec_gyro,
        sizing_mode="stretch_width",
        styles={'background': colors['neutral space']}
    )


# Set the directory to look for CSV files
data_dir = r"/2. Sensor Measurement/Collected Data"
csv_files = sorted([os.path.join(data_dir, f) for f in os.listdir(data_dir) if f.lower().endswith('.csv')])
if not csv_files:
    csv_files = [
        r"C:\Users\Louis\MSc-Thesis-Louis\4. Feature Exctraction\Dashboard\26Hz_60.00min_2025-02-27_12-03-38.csv"]

file_selector = pn.widgets.Select(name="Select a CSV file", options=csv_files, value=csv_files[0])
dashboard_panel = pn.bind(update_dashboard, file_name=file_selector)

header = pn.Row(
    pn.pane.PNG('ESA_LOGO.png', width=200),
    pn.pane.Markdown(
        "# ESA Analyzer Dashboard Sensor Data",
        align='center',
        styles={
            'margin-top': '30px',
            'background': colors['deep space'],
            'color': colors['neutral space'],
            'text-align': 'center',
            'width': '90%',
            'font-size': '25px',
            'border-radius': '10px'
        },
        sizing_mode='stretch_width'
    ),
    align='center',
    styles={'background': colors['deep space']},
    sizing_mode='stretch_width'
)

layout = pn.Column(
    header,
    pn.Row(file_selector, sizing_mode="stretch_width"),
    dashboard_panel,
    sizing_mode='stretch_width',
    styles={'background': colors['neutral space']}
)

template = pn.template.MaterialTemplate(
    title="ESA Analyzer Dashboard Sensor Data",
    header_background=colors['deep space'],
    main=[layout]
)

pn.serve(template)
