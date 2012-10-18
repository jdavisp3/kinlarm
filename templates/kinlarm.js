$(function() {

    var camera_template =  _.template($('#camera-template').html());

    function update_state() {
        $("#state").load("/state", null, function(responseText) {
            setTimeout(update_state, 3000);
        });
    };

    function setstate(state) {
        $.ajax("/setstate?" + state)
    }

    $(document).ready(function() {
        $.ajax("/num_devices")
            .success(function(numstr) {
                for (var i = parseInt(numstr); i--;) {
                    $('#cameras').append(camera_template({device_num: i}));
                }
            });
    });

});
