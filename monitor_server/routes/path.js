var express = require('express');
var router = express.Router();

/* GET home page. */
router.get('/waypoint', function(req, res, next) {
  res.render('waypoint', { name: "waypoints"});
});

router.get('/savepath', function(req, res, next) {
  res.render('savepath', { name: "Save Path"});
});

module.exports = router;
