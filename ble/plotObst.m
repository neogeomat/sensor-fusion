% PLOTOBST Plot obstacles.
%
%	[H] = PLOTOBST(POLYGONS, PCOLORT1, PCOLORT2, HO).
%
%   See Also LOADZONE and PLOTTHEMALL.
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

function [h] = plotObst(polygons, pColorT1, pColorT2, ho)

    if (exist('h', 'var'))
        h = figure(ho);
    else
        h = figure;
    end
    hold on;
    for i = (1:size(polygons,1))
        polygon = polygons{i,1};
        if (polygons{i,2} == 1)
            plot(polygon(:,1), polygon(:,2), 'color', pColorT1);
        else
            plot(polygon(:,1), polygon(:,2), 'color', pColorT2);
        end
    end
    hold off;
end