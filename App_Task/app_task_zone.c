#include "app_task_zone.h"
#include "app_task_config.h"

static const char *zone_diagnostic[2] = {"not_checked", "not_checked"}; // 仅记录红/蓝分区推导原因。

const char *App_Task_Zone_LastReason(uint8_t supply_type)
{
    return zone_diagnostic[supply_type == RESCUE_OBJECT_BLUE_SUPPLY_ZONE ? 1U : 0U];
}

static uint8_t Zone_BoxValid(const RescueScene *scene, const RescueObject *box)
{
    return box->width != 0U && box->height != 0U
           && box->cx >= box->width / 2U && box->cy >= box->height / 2U
           && (uint32_t)box->cx - box->width / 2U + box->width <= scene->image_width
           && (uint32_t)box->cy - box->height / 2U + box->height <= scene->image_height;
}

static uint16_t Zone_Left(const RescueObject *box)
{
    return box->cx - box->width / 2U;
}

static uint16_t Zone_Top(const RescueObject *box)
{
    return box->cy - box->height / 2U;
}

static uint16_t Zone_Clip(uint16_t value, uint16_t left, uint16_t right)
{
    return value < left ? left : (value > right ? right : value);
}

static uint8_t Zone_MakeBox(RescueObject *out, uint8_t type, uint8_t confidence,
                         uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)
{
    if(right <= left || bottom <= top){ return 0; } // 只输出有实际宽高的交集，不产生负数或零宽框。
    *out = (RescueObject){0};
    out->type = type;
    out->confidence = confidence;
    out->width = right - left;
    out->height = bottom - top;
    out->cx = left + out->width / 2U;
    out->cy = top + out->height / 2U;
    return 1;
}

uint8_t App_Task_Zone_Derive(const RescueScene *scene, RescueObject *zones,
                            uint8_t capacity)
{
    uint8_t count = 0;
    zone_diagnostic[0] = zone_diagnostic[1] = "zone_not_found";
    uint8_t colour;
    const RescueObject *line = NULL;
    uint8_t line_count = 0;
    if(scene == NULL || zones == NULL
       || capacity < APP_TASK_DERIVED_ZONE_MAX || scene->object_count > RESCUE_MAX_OBJECTS
       || scene->image_width == 0U || scene->image_height == 0U
       || scene->flags != RESCUE_SCENE_IMAGE_VALID){ zone_diagnostic[0] = zone_diagnostic[1] = "invalid_zone_input"; return 0; }

    // type14是混合检测列表中的分界线标记，不再检查线的位置、形状或是否属于某个区。
    for(uint8_t i = 0; i < scene->object_count; ++i){
        const RescueObject *object = &scene->objects[i];
        if(object->type == RESCUE_OBJECT_SAFE_ZONE_DIVIDER){ line = object; ++line_count; }
    }
    if(line_count != 1U){
        zone_diagnostic[0] = zone_diagnostic[1] = line_count == 0U ? "divider_missing" : "divider_multiple";
        return 0;
    }
    if(!Zone_BoxValid(scene, line)){ zone_diagnostic[0] = zone_diagnostic[1] = "divider_invalid_box"; return 0; }
    if(line->confidence < APP_TASK_CONFIDENCE_MIN){ zone_diagnostic[0] = zone_diagnostic[1] = "divider_low_confidence"; return 0; }

    for(colour = 0; colour < 2U; ++colour){
        uint8_t whole_type = colour == 0U ? RESCUE_OBJECT_RED_SAFE_ZONE : RESCUE_OBJECT_BLUE_SAFE_ZONE;
        uint8_t supply_type = colour == 0U ? RESCUE_OBJECT_RED_SUPPLY_ZONE : RESCUE_OBJECT_BLUE_SUPPLY_ZONE;
        const RescueObject *whole = NULL;
        uint8_t whole_count = 0;
        uint8_t direct = 0;
        uint8_t direct_supply = 0; // 只区分诊断，不影响原直接标签优先规则。
        uint8_t i;
        for(i = 0; i < scene->object_count; ++i){
            const RescueObject *object = &scene->objects[i];
            if(object->type == supply_type || object->type == supply_type + 1U){ direct = 1; }
            if(object->type == supply_type){ direct_supply = 1; }
            if(object->type == whole_type){ whole = object; ++whole_count; }
        }
        if(direct){ zone_diagnostic[colour] = direct_supply ? "direct_zone" : "only_casualty_zone"; continue; }
        if(whole_count != 1U){ zone_diagnostic[colour] = whole_count == 0U ? "whole_zone_missing" : "whole_zone_multiple"; continue; }
        if(!Zone_BoxValid(scene, whole)){ zone_diagnostic[colour] = "whole_zone_invalid_box"; continue; }
        if(whole->confidence < APP_TASK_CONFIDENCE_MIN){ zone_diagnostic[colour] = "whole_zone_low_confidence"; continue; }

        uint16_t left = Zone_Left(whole);
        uint16_t top = Zone_Top(whole);
        uint16_t right = left + whole->width;
        uint16_t bottom = top + whole->height;
        /* 整体框包括紫色围栏；按名义围栏比例内缩，另排除分界线整个宽度。 */
        uint16_t inset_x = (uint16_t)(whole->width * APP_TASK_ZONE_INSET_X);
        uint16_t inset_y = (uint16_t)(whole->height * APP_TASK_ZONE_INSET_Y);
        uint16_t line_left = Zone_Left(line);
        uint16_t line_right = line_left + line->width;
        left += inset_x;
        right -= inset_x;
        top += inset_y;
        bottom -= inset_y;
        // 计算两侧区间与安全区内部的交集；线在框外也不因此拒绝整个安全区。
        line_left = Zone_Clip(line_left, left, right);
        line_right = Zone_Clip(line_right, left, right);
        uint8_t confidence = whole->confidence < line->confidence ? whole->confidence : line->confidence;
        uint8_t supply_made = Zone_MakeBox(&zones[count], supply_type, confidence, left, top, line_left, bottom);
        count += supply_made;
        zone_diagnostic[colour] = supply_made ? "ok" : "supply_empty_after_clip";
        count += Zone_MakeBox(&zones[count], supply_type + 1U, confidence, line_right, top, right, bottom);
    }
    return count;
}
