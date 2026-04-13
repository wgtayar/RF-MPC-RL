function append_csv_row(csvFile, rowStruct)
    T = struct2table(rowStruct);
    if exist(csvFile, 'file')
        writetable(T, csvFile, 'WriteMode', 'append');
    else
        writetable(T, csvFile);
    end
end