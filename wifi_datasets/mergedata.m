function [mergeddata] = mergedata(data,merge_size)
    % Merging 13 points on same location
    % -----------------------
    h=1:length(data.rss)/merge_size;
    mergeddata.coords = zeros(length(h),3);
    mergeddata.rss = zeros(length(h),size(data.rss,2));
    for e = h
        mergeddata.coords(e,:) = mean(data.coords((e-1)*merge_size+2:e*merge_size,:),'omitnan');
        mergeddata.rss(e,:) = mean(data.rss((e-1)*merge_size+2:e*merge_size,:),'omitnan');
    end
end