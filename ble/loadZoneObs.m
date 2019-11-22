% LOADZONEOBS Loads polygons describing the obstacles found inside the
% collection area.
%
%	[POLYGONS] = LOADZONEOBS(ZONE, DATAFOLDER) returns the cell array
%	POLYGONS where each cell contains a polygons that represents an
%	obstacle found in the collection area of ZONE. The information is
%	loaded from folder DATAFOLDER.
%
%   See Also GETFILTERDEFS, LOADZONE, LOADZONERSS and LOADZONEDEP.
%
%   Copyright © 2018 Universitat Jaume I (UJI)

% Permission is hereby granted, free of charge, to any person obtaining a copy of
% this software and associated documentation files (the "Software"), to deal in
% the Software without restriction, including without limitation the rights to
% use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
% of the Software, and to permit persons to whom the Software is furnished to do
% so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

function [polygons] = loadZoneObs(file)  

    data = csvread([file], 1, 0);
        
    % id,x,y,type
    ids = unique(data(:,1));
    polygons = cell(numel(ids),2);
    for i = (1:numel(ids))
        sel = data(:,1) == ids(i);
        x = data(sel,2);
        y = data(sel,3);
        types = data(sel,4);
        b = convhull(x,y);
        polygons{i,1} = [x(b),y(b)];
        polygons{i,2} = types(1);
    end
end