function collapsible(elm){
      elm.classList.toggle("active");
      var content = elm.nextElementSibling;
      if (content.style.maxHeight){
        content.style.maxHeight = null;
      } else {
        content.style.maxHeight = content.scrollHeight + "px";
      } 
}
