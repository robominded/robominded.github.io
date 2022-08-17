function collapsible(elm){
  // var coll = document.getElementsByClassName("collapsible");
  // var i;

  // for (i = 0; i < coll.length; i++) {
  //   coll[i].addEventListener("click", function() {
      elm.toggle("active");
      var content = elm.nextElementSibling;
      if (content.style.maxHeight){
        content.style.maxHeight = null;
      } else {
        content.style.maxHeight = content.scrollHeight + "px";
      } 
    // });
  // }
}