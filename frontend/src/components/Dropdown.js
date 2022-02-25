import React from 'react'

function Dropdown(props) {
    return (
        <div className="relative mb-1 hover:bg-blue-400 rounded-lg">
        <select onChange={props.onChange} className="block appearance-none w-full bg-transparent border-gray-200 text-white py-3 px-4 pr-8 leading-tight focus:outline-none font-bold">
            {props.options.map((option, ind) =>
                props.selected === ind ?
                <option key={option} selected>{option}</option>
                : <option key={option}>{option}</option>
            )}
        </select>
        <div className="absolute flex inset-y-0 items-center px-3 right-0 font-bold text-white pointer-events-none">
            <svg className="fill-current h-4 w-4" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 20"><path d="M9.293 12.95l.707.707L15.657 8l-1.414-1.414L10 10.828 5.757 6.586 4.343 8z"></path></svg>
        </div>
    </div>
    )
}

export default Dropdown
