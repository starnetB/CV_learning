!function(){function e(){return!!window.navigator.userAgent.toLowerCase().match(/(phone|pad|pod|iphone|ipod|ios|ipad|android|mobile|blackberry|iemobile|mqqbrowser|juc|fennec|wosbrowser|browserng|webos|symbian|windows phone)/i)}function n(){return!!window.navigator.userAgent.toLowerCase().match(/(csdn)/i)}function o(){var o=null;o=e()?document.querySelectorAll('[class^="container-fluid container-fluid-flex container-"]'):n()?document.querySelectorAll('[class^="recommend_item type_"]'):document.querySelectorAll('[class^="recommend-item-box type_"]'),null!==o&&o.length<=5&&$.get("https://statistic.csdn.net/blog/recommend?count="+o.length+"&articleId="+articleId)}o()}();