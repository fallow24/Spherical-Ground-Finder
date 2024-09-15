function [] = evalScenarioOBC(max_entries, gt)
  printf("--------- EVALUATION ---------\n");
  filename="obc";
  printf("File: %s\n", filename);

  plot_times_mat = zeros(2, 6, 8); # zeros(# filter-subcloud options, # of plane alg, elems per stack)
  plot_tot_times_mat = zeros(2, 6, 4); # same dimension
  plot_total_times = zeros(2, 6); # zeros(# filter-subcloud options, # of plane alg)
  fail_rates = zeros(2, 6);
  mean_abs_angle_error = zeros(2, 6);
  median_abs_angle_error = zeros(2, 6);
  variances = zeros(2, 6);

  % Filter-Subcloud 1: geo-geo
  files_sc1 = [["lsf/geo-geo-", filename, ".csv"]; ["pca/geo-geo-", filename, ".csv"]; ["ran/geo-geo-", filename, ".csv"]; ["ran/geo-geo-lsf_", filename, ".csv"]; ["rht/geo-geo-", filename, ".csv"]; ["rht2/geo-geo-", filename, ".csv"]];
  files_sc1 = cellstr(files_sc1);

  % Filter-Subcloud 2: none-geo
  filename="obc2";
  files_sc2 = [["lsf/none-geo-", filename, ".csv"]; ["pca/none-geo-", filename, ".csv"]; ["ran/none-geo-", filename, ".csv"]; ["ran/none-geo-lsf_", filename, ".csv"]; ["rht/none-geo-", filename, ".csv"]; ["rht2/none-geo-", filename, ".csv"]];
  files_sc2 = cellstr(files_sc2);

  filename="obc";

  files = [files_sc1, files_sc2];

  fail_counts = zeros(6, 1);

  % Label for Filter-Subcloud in time plots
  labels_text = [["geo-geo"]; ["none-geo"]];
  labels_text = cellstr(labels_text);

  for j=1:2
    abs_err_angles = zeros(6, max_entries(j));
    printf("-- Case: %s\n", char(labels_text(j, 1)));
    for i=1:6
      % Read file
      [times, tot_times, total_time, nx, ny, nz, plane_count] = readCSV(char(files(i, j)));

      % Calculate avg. times [ms]
      avg_times = takeMeanTimes(times);
      avg_tot_times = takeMeanTimes(tot_times);
      avg_total_time = takeMeanTimes(total_time);
      plot_times_mat(j,i,:) = avg_times;
      plot_tot_times_mat(j,i,:) = avg_tot_times;
      plot_total_times(j, i) = avg_total_time;

      % Calculate mean absolute angle error and plot angle over iterations [ms]
      index = max_entries(j) - length(nx) + 1;
      [mean_abs_angle_error(j,i), median_abs_angle_error(j,i), abs_err_angles(i,index:end), fail_counts(i)] = absAngleError(nx, ny, nz, gt, plane_count);
      variances(j,i) = stdVariance(nx, ny, nz, gt, plane_count, mean_abs_angle_error(j,i));
    endfor
    figure(j);
    plotError(abs_err_angles);
    abs_err_angles = zeros(6, max_entries(j));
    title(char(labels_text(j, 1)));

    fail_rates(j,:) = fail_counts / max_entries(j) * 100.0; # [%]

    # Output
    disp("Fail counts: LSF - PCA - RAN_PCA - RAN_LSF - RHT - RHT2");
    disp(fail_counts);
    disp("");
  endfor
  %

  % ------------- write to text file for latex  -------------
  ## Time file
  fid = fopen(["results_tables/", filename, "_times.txt"], 'w+');

  text_lab1 = [["& \\textbf{Downsample [ms]} & "]; ["& \\textbf{Build Tree [ms]} & "]; ["& \\textbf{Radius Search [ms]} & "]; ["& \\textbf{Delete [ms]} & "];
          ["& \\textbf{Build Tree [ms]} & "]; ["& \\textbf{Cheese Search [ms]} & "]; ["& \\textbf{Delete [ms]} & "]; ["& \\textbf{Plane [ms]} & "]];
  text_lab1 = cellstr(text_lab1);

  text_lab = [text_lab1, text_lab1];

  text_tab = [["\\multirow{6}{*}{\\rotatebox[origin=c]{90}{geo-geo}}"]; ["\\multirow{5}{*}{\\rotatebox[origin=c]{90}{none-geo}}"]];
  text_tab = cellstr(text_tab);

  for j=1:2
    fprintf(fid, '%s ', char(text_tab(j)));
    fprintf(fid, '\n');
    for k=1:8
      fprintf(fid, '%s ', char(text_lab(k,j)));
      avg_tmp = 0.0;
      for i=1:6
        fprintf(fid, '%.3f & ', plot_times_mat(j,i,k));
        avg_tmp = avg_tmp + plot_times_mat(j,i,k);
      endfor
      avg_tmp = avg_tmp / 6;
      fprintf(fid, "\\textbf{%.3f} \\\\*", avg_tmp);
      avg_tmp = 0.0;
      fprintf(fid, '\n');
    endfor
    %
    fprintf(fid, "\\cline{2-8}");
    fprintf(fid, '\n');
    fprintf(fid, "& \\textbf{Total [ms]} & ");
    for i=1:6
      fprintf(fid, '%.3f & ', plot_total_times(j,i));
      avg_tmp = avg_tmp + plot_total_times(j,i);
    endfor
    avg_tmp = avg_tmp / 6;
    fprintf(fid, "\\textbf{%.3f} \\\\*", avg_tmp);
    avg_tmp = 0.0;
    fprintf(fid, '\n');
    fprintf(fid, "& \\textbf{Fail Rate [perc]} & ");
    for i=1:6
      fprintf(fid, '%.3f & ', fail_rates(j,i));
      avg_tmp = avg_tmp + fail_rates(j,i);
    endfor
    avg_tmp = avg_tmp / 6;
    fprintf(fid, "\\textbf{%.3f} \\\\", avg_tmp);
    avg_tmp = 0.0;
    fprintf(fid, '\n');
    fprintf(fid, "\\hline");
    fprintf(fid, '\n');
  endfor
  fclose(fid);

  ## Mean Angle error and variance file
  fid = fopen(["results_tables/", filename, "_error.txt"], 'w+');
  text_lab = [["& \\textbf{\\gls{mae} [deg]} & "]; ["& \\textbf{Variance [deg$^2$]} & "]; ["& \\textbf{\\gls{medae} [deg]} & "]];
  text_lab = cellstr(text_lab);

  text_tab = [["\\multirow{2}{*}{geo-geo}"]; ["\\multirow{2}{*}{none-geo}"]];
  text_tab = cellstr(text_tab);

  for j=1:2
    fprintf(fid, '%s ', char(text_tab(j)));
    fprintf(fid, '\n');
    # Mean Absolute Error
    fprintf(fid, '%s ', char(text_lab(1)));
    for i=1:6
      fprintf(fid, '%.3f', mean_abs_angle_error(j,i));
      if i != 6
        fprintf(fid, ' & ');
      else
        fprintf(fid, ' \\\\');
      endif
    endfor
    fprintf(fid, '\n');
    %
    # Median Absolute Error
    fprintf(fid, '%s ', char(text_lab(3)));
    for i=1:6
      fprintf(fid, '%.3f', median_abs_angle_error(j,i));
      if i != 6
        fprintf(fid, ' & ');
      else
        fprintf(fid, ' \\\\');
      endif
    endfor
    fprintf(fid, '\n');
    %
    # Variance
    fprintf(fid, '%s ', char(text_lab(2)));
    for i=1:6
      fprintf(fid, '%.3f', variances(j,i));
      if i != 6
        fprintf(fid, ' & ');
      else
        fprintf(fid, ' \\\\');
      endif
    endfor
    fprintf(fid, '\n');
    fprintf(fid, "\\hline");
    fprintf(fid, '\n');
  endfor
  fclose(fid);


  % ------------- time plots  -------------
  % Label for Filter-Subcloud in time plots
  labels = {"geo-geo", "none-geo"};

  figure(101);
  plotBarStackGroups(plot_times_mat, labels, 2.0);
  plot_name = ["results_plots/", filename, "_times_full"];
  print(plot_name, "-depslatex");

  figure(102);
  plotBarStackGroups(plot_tot_times_mat, labels, 2.0);
  plot_name = ["results_plots/", filename, "_times_reduced"];
  print(plot_name, "-depslatex");

endfunction
%
