import { React, useState, useEffect, useReducer } from 'react'
import Sidebar from '../components/Sidebar/Sidebar';
import {
  BrowserRouter as Router,
  Switch,
  Route
} from "react-router-dom";
import SubPage from './SubPage';
import { CAMERA_GROUPS_LAYOUT, DASHBOARD_LAYOUT, PROFILE_LAYOUT } from '../data/main_view_layouts';
import { UserContext } from '../components/Contexts/UserContext';
import { getUserContext } from '../data/user_data';
import { GlobalAppReducer, initialState } from '../components/Contexts/Reducer';
import { StateContext } from '../components/Contexts/StateContext';
import Loading from '../components/InfoWidgetTypes/Loading';
import { RefreshIcon } from '@heroicons/react/solid';

const routes = [
  {
    path: "/",
    exact: true,
    main: () => <SubPage layout={DASHBOARD_LAYOUT}></SubPage>
  },
  {
    path: "/profile",
    main: () => <SubPage layout={PROFILE_LAYOUT}></SubPage>
  },
  {
    path: "/cameragroups",
    main: () => <SubPage layout={CAMERA_GROUPS_LAYOUT}></SubPage>
  }
];

function getPage(userContext, state) {
  if (userContext == null || state.isLoading) {
    return <div className="flex flex-col h-screen p-6 bg-steel-grey overscroll-contain">
      <Loading loadingMessage={state.loadingMessage}></Loading>
    </div>
  } else {
    return (
      <div className="flex flex-col h-screen p-6 bg-steel-grey overscroll-contain">
        <div className="flex flex-row max-w-full">
          <p className="text-white text-lg mb-6">People Counter</p>
          <div className="flex-grow mb-6"></div>
          <p className='text-gray-500 text-base mr-2'>Data last refreshed </p>
          <RefreshIcon className="text-white h-6 w-6 hover:bg-blue-400 rounded-md p-1" onClick={() => {
          }}></RefreshIcon>
        </div>
        <div className="flex h-full">
          <Sidebar></Sidebar>
          <Switch>
            {routes.map((route, index) => (
              <Route
                key={index}
                path={route.path}
                exact={route.exact}
                children={<route.main />}
              />
            ))}
          </Switch>
        </div>
      </div>
    )
  }
}

/*
 * Hash values compare with the server and client to update data
 * Store data locally and compare hash periodically to see if the data needs updating
 */
export default function MainPage() {
  const [userContext, setUserContext] = useState(null)
  const [state, dispatch] = useReducer(GlobalAppReducer, initialState)

  useEffect(() => {
    getUserContext((ret) => {
      setUserContext(ret)
    })
  }, [])
  // Find and load user context, if not found then put out a loading until userContext is returned from the backend
  return <Router>
    <UserContext.Provider value={userContext}>
      <StateContext.Provider value={[state, dispatch]}>
        {getPage(userContext, state)}
      </StateContext.Provider>
    </UserContext.Provider>
  </Router>
}