function [] = data_plot(data, labels, rows, cols, titles)
%DATA_PLOT prints data given a data matrix stored with data_store function
%
% data   : data matrix computed through data_store function
% labels : label for each entry [row] of data

for i=1:size(labels)
    subplot(rows, cols, i);
    plot(data(i, :));
    xlabel(labels(i, 1));
    ylabel(labels(i, 2));
    title(titles(i, :));
    grid on;
end
end

