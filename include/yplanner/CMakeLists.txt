set(public_headers_
  yplanner.h
)


set(headers_ 
  ${publish_headers_}
)


set(hdrs_)
set(public_hdrs_)

foreach(a IN ITEMS ${headers_})
  set(hdrs_ ${hdrs_} include/${PROJECT_NAME}/${a})
endforeach()

foreach(a IN ITEMS ${public_headers_})
  set(public_hdrs_ ${public_hdrs_} include/${PROJECT_NAME}/${a})
endforeach()

set(public_hdrs ${public_hdrs_} PARENT_SCOPE)
set(hdrs ${hdrs_} PARENT_SCOPE)
