var express = require('express');
var router = express.Router();

/* GET home page. */
router.get('/logger', function(req, res, next) {
  res.render('gps_logger', { name: 'GPS Logger' });
});

module.exports = router;
