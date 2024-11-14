% Create timeseries objects for each coordinate
x_ts = timeseries(x_data, time_data, 'Name', 'X');
y_ts = timeseries(y_data, time_data, 'Name', 'Y');
z_ts = timeseries(z_data, time_data, 'Name', 'Z');

% Create a Dataset object for each coordinate
x_dataset = Simulink.SimulationData.Dataset;
y_dataset = Simulink.SimulationData.Dataset;
z_dataset = Simulink.SimulationData.Dataset;

% Add the timeseries data to the Dataset
x_dataset = x_dataset.addElement(x_ts);
y_dataset = y_dataset.addElement(y_ts);
z_dataset = z_dataset.addElement(z_ts);

% Save each Dataset to .mat files
save('x_coordinates.mat', 'x_dataset');
save('y_coordinates.mat', 'y_dataset');
save('z_coordinates.mat', 'z_dataset');
% Create timeseries objects for each coordinate
x_ts = timeseries(x_data, time_data, 'Name', 'X');
y_ts = timeseries(y_data, time_data, 'Name', 'Y');
z_ts = timeseries(z_data, time_data, 'Name', 'Z');

% Create a Dataset object for each coordinate
x_dataset = Simulink.SimulationData.Dataset;
y_dataset = Simulink.SimulationData.Dataset;
z_dataset = Simulink.SimulationData.Dataset;

% Add the timeseries data to the Dataset
x_dataset = x_dataset.addElement(x_ts);
y_dataset = y_dataset.addElement(y_ts);
z_dataset = z_dataset.addElement(z_ts);

% Save each Dataset to .mat files
save('x_coordinates.mat', 'x_dataset');
save('y_coordinates.mat', 'y_dataset');
save('z_coordinates.mat', 'z_dataset');
% Create timeseries objects for each coordinate
x_ts = timeseries(x_data, time_data, 'Name', 'X');
y_ts = timeseries(y_data, time_data, 'Name', 'Y');
z_ts = timeseries(z_data, time_data, 'Name', 'Z');

% Create a Dataset object for each coordinate
x_dataset = Simulink.SimulationData.Dataset;
y_dataset = Simulink.SimulationData.Dataset;
z_dataset = Simulink.SimulationData.Dataset;

% Add the timeseries data to the Dataset
x_dataset = x_dataset.addElement(x_ts);
y_dataset = y_dataset.addElement(y_ts);
z_dataset = z_dataset.addElement(z_ts);

% Save each Dataset to .mat files
save('x_coordinates.mat', 'x_dataset');
save('y_coordinates.mat', 'y_dataset');
save('z_coordinates.mat', 'z_dataset');
