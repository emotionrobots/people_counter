import React from 'react'

export default function NavItem(props) {
    return (
        <div className="flex flex-row group font-bold text-lg text-white ml-2 my-auto group-hover:text-blue-300 justify-start items-center">
            <props.icon className="h-10 w-10 text-white mb-2 mt-2 group-hover:text-blue-300"></props.icon>
            <div className="group-hover:text-blue-300 align-center pl-2">{props.desc}</div>
        </div>
    )
}
