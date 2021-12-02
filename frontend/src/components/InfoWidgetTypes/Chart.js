import { React, useState, createRef, useRef } from 'react'
import { VscSettings, VscSettingsGear } from 'react-icons/vsc'
import { XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Legend, BarChart, Bar } from 'recharts'
import DatePicker from 'react-datepicker'
import SettingsModal from '../Modals/SettingsModal'
import { createPortal } from 'react-dom'

const DEBUG_DATA = [
    { name: '1:00', personExit: 2, personEnter: 5, peopleInside: 4 },
    { name: '2:00', personExit: 5, personEnter: 8, peopleInside: 6 },
    { name: '3:00', personExit: 7, personEnter: 6, peopleInside: 4 },
    { name: '4:00', personExit: 4, personEnter: 9, peopleInside: 3 },
]

const KEYS = {
    'personExit': '# of people exited',
    'personEnter': '# of people entered',
    'peopleInside': '# of people inside',
}

const GRAPH_COLORS = [
    '#8884d8',
    '#82ca9d',
    '#ca829b',
    '#cacc83',
]

const getInitialArrayOfLen = (len) => {
    let arr = []
    for (let index = 0; index < len; index++) {
        arr[index] = true
    }

    return arr
}

/**
 * Chart info widget component renders data based on 
 * @param {ChartContext} props 
 * @returns Chart info widget body
 */
export default function Chart(props) {
    const [showChartConfigModal, setShowChartConfigModal] = useState(false)
    const [checkedYAxis, setCheckedYAxis] = useState(getInitialArrayOfLen(Object.keys(KEYS).length))
    const [startDate, setStartDate] = useState(new Date())
    const [endDate, setEndDate] = useState(new Date().setMonth(startDate.getMonth() + 1))
    const checkboxRefs = useRef([]);

    if (checkboxRefs.current.length !== checkedYAxis.length) {
        // add or remove refs
        checkboxRefs.current = Array(checkedYAxis.length).fill().map((_, i) => checkboxRefs.current[i] || createRef());
    }

    return (
        <div className='flex relative h-full w-full p-4'>
            {/* <div className='flex flex-col w-60'>
                <div className='font-bold text-white text-xl'>Data Configuration</div>
            </div> */}
            <ResponsiveContainer>
                <BarChart data={DEBUG_DATA} margin={{ top: 5, right: 10, bottom: 0, left: -35 }}>
                    {/* <defs>
                        {
                            checkedYAxis.map((val, ind) => {
                                if (val)
                                    return <linearGradient key={ind} id={Object.keys(KEYS)[ind] + 'color'} x1="0" y1="0" x2="0" y2="1">
                                        <stop offset="5%" stopColor={GRAPH_COLORS[ind]} stopOpacity={0.8} />
                                        <stop offset="95%" stopColor={GRAPH_COLORS[ind]} stopOpacity={0} />
                                    </linearGradient>
                                else
                                    return <div key={ind} />
                            })
                        }
                    </defs> */}
                    <XAxis dataKey="name" />
                    <YAxis />
                    <Legend />
                    <CartesianGrid vertical={false} strokeDasharray='2 2' />
                    <Tooltip />
                    {
                        checkedYAxis.map((val, ind) => {
                            if (val)
                                return <Bar key={ind} name={Object.values(KEYS)[ind]} dataKey={Object.keys(KEYS)[ind]} fill={GRAPH_COLORS[ind % GRAPH_COLORS.length]} />
                            else
                                return <div key={ind} />
                        })
                    }
                </BarChart>
            </ResponsiveContainer>
            <div className='p-1 absolute bottom-2 right-2 text-white text-xl font-bold rounded-md hover:bg-blue-400' onClick={() => {
                setShowChartConfigModal(true)
            }}>
                <VscSettings />
            </div>

            <SettingsModal
                open={showChartConfigModal}
                setOpen={setShowChartConfigModal}
                modalTitle={'Chart Configuration'}
                icon={VscSettingsGear}
                iconColor='blue'
                onSave={() => {
                    let tmp = []
                    checkboxRefs.current.forEach(element => {
                        tmp.push(element.current.checked)
                    });
                    setCheckedYAxis(tmp)
                }}>
                <div className="text-gray-500 mb-1">Configure the data lines shown on the graph.</div>
                <div className="max-h-48 overflow-scroll">
                    {Object.values(KEYS).map((val, ind) => {
                        return <Checkbox key={ind}
                            checkboxLabel={val}
                            checkboxID={Object.keys(KEYS)[ind]}
                            ticked={checkedYAxis[ind]}
                            inputRef={checkboxRefs.current[ind]} />
                    })}
                </div>
                <div className="text-gray-500 my-2">Select a date range the graph should display.</div>
                <div className="flex flex-row mt-2">
                    <div className="">
                        <DatePicker
                            selected={startDate}
                            onChange={(date) => setStartDate(date)}
                            selectsStart
                            startDate={startDate}
                            endDate={endDate}
                            maxDate={new Date()}
                            nextMonthButtonLabel=">"
                            previousMonthButtonLabel="<"
                            popperContainer={({children}) => {
                                return createPortal(children, document.body)
                            }}
                        />
                    </div>
                    <div >
                        <DatePicker
                            selected={endDate}
                            onChange={(date) => setEndDate(date)}
                            selectsEnd
                            startDate={startDate}
                            endDate={endDate}
                            minDate={startDate}
                            maxDate={new Date()}
                            nextMonthButtonLabel=">"
                            previousMonthButtonLabel="<"
                            popperContainer={({children}) => {
                                return createPortal(children, document.body)
                            }}
                        />
                    </div>
                </div>
            </SettingsModal>
        </div>
    )
}

function Checkbox(props) {
    const [ticked, setTicked] = useState(props.ticked)

    return <div className="flex items-center p-1">
        <input type="checkbox" ref={props.inputRef} id={props.checkboxID} name={props.checkboxID} checked={ticked} onChange={(e) => {
            setTicked(e.target.checked)
        }}
            className="opacity-0 absolute h-8 w-8" />
        <div className={(ticked ? 'bg-blue-600 border-opacity-0' : 'bg-white') + " border-2 rounded-md w-6 h-6 flex flex-shrink-0 justify-center items-center mr-2 focus-within:border-blue-500"}>
            <svg className={"fill-current " + (ticked ? 'block' : 'hidden') + " w-3 h-3 text-white font-bold pointer-events-none"} version="1.1" viewBox="0 0 17 12" xmlns="http://www.w3.org/2000/svg">
                <g fill="none" fillRule="evenodd">
                    <g transform="translate(-9 -11)" fill="#FFFFFF" fillRule="nonzero">
                        <path d="m25.576 11.414c0.56558 0.55188 0.56558 1.4439 0 1.9961l-9.404 9.176c-0.28213 0.27529-0.65247 0.41385-1.0228 0.41385-0.37034 0-0.74068-0.13855-1.0228-0.41385l-4.7019-4.588c-0.56584-0.55188-0.56584-1.4442 0-1.9961 0.56558-0.55214 1.4798-0.55214 2.0456 0l3.679 3.5899 8.3812-8.1779c0.56558-0.55214 1.4798-0.55214 2.0456 0z" />
                    </g>
                </g>
            </svg>
        </div>
        <label htmlFor={props.checkboxID} className="select-none">{props.checkboxLabel}</label>
    </div>
}