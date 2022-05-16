var express = require('express');
var router = express.Router();

/* GET home page. */
router.get('/', function(req, res, next) {
    var name = req.query.name
    var type = req.query.type
    
    res.render('topic', { name: name, type: type });
});

module.exports = router;
