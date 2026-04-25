#import "/rmcs_notebook/template/template.typ": ilm

#show: ilm.with(
  title: [Deformable Infantry \ 机构、解算与控制],
  author: "RMCS Draft",
  date: datetime.today(),
  abstract: [
    本文档整理 deformable infantry 系列底盘的机构设计、几何解算、关节控制与主动悬挂问题。
    文档重点不在代码接口罗列，而在于给出统一符号、明确假设、建立坐标系，并对关键控制量进行可复用的数学推导。
  ],
  preface: [
    #align(center + horizon)[
      本草案参考 #raw("rmcs_notebook") 的 Typst 组织方式，
      目标是形成一套面向 deformable infantry 的数学推导型技术文档。
    ]
  ],
  table-of-contents: outline(depth: 3),
  figure-index: (enabled: true),
  table-index: (enabled: true),
  listing-index: (enabled: true),
)

= Deformable Infantry 总览
#include "chapters/introduction.typ"

= Deformable Infantry V1：丝杆机构
#include "chapters/v1_screw.typ"

= Deformable Infantry V2：直驱关节控制
#include "chapters/v2_direct_drive.typ"

= Deformable Infantry：主动悬挂
#include "chapters/active_suspension.typ"

= Deformable Infantry Omni：底盘解算
#include "chapters/omni.typ"

= Deformable Infantry V2：舵轮解算
#include "chapters/v2_steering.typ"
