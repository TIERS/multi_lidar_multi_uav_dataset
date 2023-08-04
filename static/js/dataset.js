String.prototype.format = function () {
    var i = 0, args = arguments;
    return this.replace(/{}/g, function () {
      return typeof args[i] != 'undefined' ? args[i++] : '';
    });
};

for(let i = 1; i <= 4; i++)
{
    d3.csv('./static/csv/HolybroStdn0{}.csv'.format(i), function(err, rows){
        function unpack(rows, key) {
            return rows.map(function(row)
            { return row[key]; }); }
  
        var x = unpack(rows , 'x');
        var y = unpack(rows , 'y');
        var z = unpack(rows , 'z');
        Plotly.newPlot('stdn_track_{}'.format(i), [{
            type: 'scatter3d',
            mode: 'lines',
            x: x,
            y: y,
            z: z,
            opacity: 1,
            line: {
                width: 3,
                reversescale: false
            }
        }], {
            height: 640,
            scene : {
                aspectmode: "data",
           },
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
        Plotly.newPlot('tello_track_{}'.format(i), [{
            type: 'scatter3d',
            mode: 'lines',
            x: x,
            y: y,
            z: z,
            opacity: 1,
            line: {
                width: 3,
                reversescale: false
            }
        }], {
            height: 640,
            scene : {
                aspectmode: "data",
           },
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
        Plotly.newPlot('autel_track_{}'.format(i), [{
            type: 'scatter3d',
            mode: 'lines',
            x: x,
            y: y,
            z: z,
            opacity: 1,
            line: {
                width: 3,
                reversescale: false
            }
        }], {
            height: 640,
            scene : {
                aspectmode: "data",
           },
        });
      });
}

for(let i = 1; i <= 5; i++)
{
    d3.csv('./static/csv/holybro_track_{}.csv'.format(i), function(err, rows){
        function unpack(rows, key) {
            return rows.map(function(row)
            { return row[key]; }); }
  
        var x = unpack(rows , 'x');
        var y = unpack(rows , 'y');
        var z = unpack(rows , 'z');
        Plotly.newPlot('holybro_track_{}'.format(i), [{
            type: 'scatter3d',
            mode: 'lines',
            x: x,
            y: y,
            z: z,
            opacity: 1,
            line: {
                width: 3,
                reversescale: false
            }
        }], {
            height: 640,
            scene : {
                aspectmode: "data",
           },
        });
      });
}
