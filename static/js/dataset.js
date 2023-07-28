String.prototype.format = function () {
    var i = 0, args = arguments;
    return this.replace(/{}/g, function () {
      return typeof args[i] != 'undefined' ? args[i++] : '';
    });
};

for(let i = 1; i <= 5; i++)
{
    d3.csv('./static/csv/holybro_track_{}.csv'.format(i), function(err, rows){
        function unpack(rows, key) {
            return rows.map(function(row)
            { return row[key]; }); }
  
        var x = unpack(rows , 'x');
        var y = unpack(rows , 'y');
        var z = unpack(rows , 'z');
        var c = unpack(rows , 'color');
        Plotly.newPlot('holybro_track_{}'.format(i), [{
            type: 'scatter3d',
            mode: 'lines',
            x: x,
            y: y,
            z: z,
            opacity: 1,
            line: {
                width: 6,
                color: c,
                reversescale: false
            }
        }], {
        height: 640
        });
      });
}

for(let i = 1; i <= 5; i++)
{
    d3.csv('./static/csv/tello_track_{}.csv'.format(i), function(err, rows){
        function unpack(rows, key) {
            return rows.map(function(row)
            { return row[key]; }); }
  
        var x = unpack(rows , 'x');
        var y = unpack(rows , 'y');
        var z = unpack(rows , 'z');
        var c = unpack(rows , 'color');
        Plotly.newPlot('tello_track_{}'.format(i), [{
            type: 'scatter3d',
            mode: 'lines',
            x: x,
            y: y,
            z: z,
            opacity: 1,
            line: {
                width: 6,
                color: c,
                reversescale: false
            }
        }], {
        height: 640
        });
      });
}

for(let i = 1; i <= 5; i++)
{
    d3.csv('./static/csv/autel_track_{}.csv'.format(i), function(err, rows){
        function unpack(rows, key) {
            return rows.map(function(row)
            { return row[key]; }); }
  
        var x = unpack(rows , 'x');
        var y = unpack(rows , 'y');
        var z = unpack(rows , 'z');
        var c = unpack(rows , 'color');
        Plotly.newPlot('autel_track_{}'.format(i), [{
            type: 'scatter3d',
            mode: 'lines',
            x: x,
            y: y,
            z: z,
            opacity: 1,
            line: {
                width: 6,
                color: c,
                reversescale: false
            }
        }], {
        height: 640
        });
      });
}
