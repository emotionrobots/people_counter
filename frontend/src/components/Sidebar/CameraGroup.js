import React from 'react'

export const CAMGROUP_COLORS = [
    'red',
    'yellow',
    'green',
    'blue',
    'indigo',
    'purple',
    'pink,'
]

function CameraGroup(props) {
    return (
        <div onClick={props.onClick} className={"flex flex-row rounded-2xl p-1 pl-4 items-center " + (!props.selected ? 'hover:bg-blue-400 cursor-pointer' : 'cursor-default')}>
            <div className={"w-2 h-2 mr-2 rounded-full bg-" + CAMGROUP_COLORS[props.id % CAMGROUP_COLORS.length] + '-500'}></div>
            <div className={(props.selected ? "text-blue-400" : "text-white") + " font-bold"}>{props.camGroup}</div>
        </div>
    )
}

export default CameraGroup
