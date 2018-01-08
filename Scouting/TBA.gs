/*
 * The Blue Alliance API
 * https://www.thebluealliance.com/apidocs
 *
 * Google Script API by Dereck Wonnacott {1596, 2586, 3538}
 */


// Fetch the rank list for an event
function ranklist(EventKey) {
   // Default values for testing
  if (EventKey == undefined) EventKey = "2017week0" 
 
  
  var list = tbaFetch("event/" + EventKey + "/rankings")
 
  var table = [];
  
  for (var i = 0; i < list.rankings.length; i++) {
    
    var d = [];
    d.push(list.rankings[i].rank);
    d.push(keytonum(list.rankings[i].team_key));
    d.push(list.rankings[i].matches_played);
    d.push(list.rankings[i].record.wins);
    d.push(list.rankings[i].record.losses);
    d.push(list.rankings[i].record.ties);
    d.push(list.rankings[i].dq);
    
    table.push(d);
  }

  return table;
}


// Fetch a match list for an event
function matchlist(EventKey) {
   // Default values for testing
  if (EventKey == undefined) EventKey = "2017week0" 
  
  var list = tbaFetch("event/" + EventKey + "/matches")
 
  // Sort the list
  function compare(a,b) {
    if (a.match_number < b.match_number) return -1;
    if (a.match_number > b.match_number) return 1;
    return 0;
  }
  list.sort(compare)
  
  var table = [];
  
    for (var i = 0; i < list.length; i++) {
      
      var item = list[i];
      
      if (item.comp_level != 'qm') continue;    // Only display Qualifying Matches
      
      var row = [];
      row.push(item.match_number);
      row.push(item.alliances.red.score);
      row.push(item.alliances.blue.score);
      row.push(keytonum(item.alliances.red.team_keys[0]));
      row.push(keytonum(item.alliances.red.team_keys[1]));
      row.push(keytonum(item.alliances.red.team_keys[2]));
      row.push(keytonum(item.alliances.blue.team_keys[0]));
      row.push(keytonum(item.alliances.blue.team_keys[1]));
      row.push(keytonum(item.alliances.blue.team_keys[2]));
      
      
      // Game-Specific
      if(list[i].score_breakdown != null) {
        row.push(JSON.stringify(list[i].score_breakdown.red));
        row.push(JSON.stringify(list[i].score_breakdown.blue));
      }
      
      table.push(row);
    }
  
  return table;
}


// Fetch a list of teams at an event
function teamlist(EventKey) {
   
  // Default values for testing
  if (EventKey == undefined) EventKey = "2017week0" 
  
  // Fetch the data from TBA
  var list = tbaFetch("event/" + EventKey + "/teams")
 
  // Sort the list
  function compare(a,b) {
    if (a.team_number < b.team_number) return -1;
    if (a.team_number > b.team_number) return 1;
    return 0;
  }
  list.sort(compare)
  
  // Reformat the data into a grid
  var table = [];
  
  for (var i = 0, len = list.length; i < len; i++) {
     var d = [];
     d.push(Number(list[i].team_number));
     d.push(list[i].nickname);
     d.push(list[i].motto);
     d.push(list[i].website);
     table.push(d);
  }
  
  // return the grid to the sheet
  return table;
}


// Fetch the JSON data from the TBA API
function tbaFetch(key) {
  
  // Default values for testing
  if (key == undefined) {
    key = "team/frc3538" 
  }
  
  // generate the TBA URL
  var url = "https://www.thebluealliance.com/api/v3/" + key;
  
  // Insert the TBA authentication Token into the header of the URL request
  // Get a token from TBA by making an account and then making a ReadKey on your account page.
  var options = {
     method         : 'get',
     contentType    : 'application/json',
     headers:{
       "X-TBA-App-Id"   : 'FRC3538:ScoutDB:0.1',
       "X-TBA-Auth-Key" : 'XXXXXXXXXXXXXXXXXXXXXX YOUR KEY HERE XXXXXXXXXXXXXXXXXXXXXXXXX'       
     }
   };
  
  // Fetch!
  var jsondata = UrlFetchApp.fetch(url, options);
  var data   = JSON.parse(jsondata.getContentText());
  
  // Return the results
  return data;
}


// Convert a TeamKey into a Team Number
function keytonum(team_key){
  if (typeof team_key === 'string' || team_key instanceof String) {
  
    // Team Number from Team Key
    var str = team_key;
    return Number(str.replace("frc", ""));
  }
  else 
    return team_key
}
